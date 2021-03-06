/*
  gcode.c - rs274/ngc parser.
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* This code is inspired by the Arduino GCode Interpreter by Mike Ellery and the NIST RS274/NGC Interpreter
   by Kramer, Proctor and Messina. */

#include "gcode.h"
#include <stdlib.h>
#include <string.h>
#include "nuts_bolts.h"
#include <math.h>
#include "settings.h"
#include "motion_control.h"
#include "spindle_control.h"
#include "errno.h"
#include "serial_protocol.h"
#include "cap_control.h"

#define MM_PER_INCH (25.4)

#define NEXT_ACTION_DEFAULT 0
#define NEXT_ACTION_DWELL 1
#define NEXT_ACTION_GO_HOME 2
#define NEXT_ACTION_MEASURE_CAP 3
#define NEXT_ACTION_MILL_GO_HOME 4
#define NEXT_ACTION_CUR_POS_IS_ORIGIN 5
#define NEXT_ACTION_TURN_OFF_ACCEL 6
#define NEXT_ACTION_TURN_ON_ACCEL 7

#define MOTION_MODE_SEEK 0 // G0 
#define MOTION_MODE_LINEAR 1 // G1
#define MOTION_MODE_CW_ARC 2  // G2
#define MOTION_MODE_CCW_ARC 3  // G3
#define MOTION_MODE_CANCEL 4 // G80

#define PATH_CONTROL_MODE_EXACT_PATH 0
#define PATH_CONTROL_MODE_EXACT_STOP 1
#define PATH_CONTROL_MODE_CONTINOUS  2

#define PROGRAM_FLOW_RUNNING 0
#define PROGRAM_FLOW_PAUSED 1
#define PROGRAM_FLOW_COMPLETED 2

#define SPINDLE_DIRECTION_CW 0
#define SPINDLE_DIRECTION_CCW 1

typedef struct {
  uint8_t status_code;

  uint8_t motion_mode;             /* {G0, G1, G2, G3, G80} */
  uint8_t inverse_feed_rate_mode;  /* G93, G94 */
  uint8_t inches_mode;             /* 0 = millimeter mode, 1 = inches mode {G20, G21} */
  uint8_t absolute_mode;           /* 0 = relative motion, 1 = absolute motion {G90, G91} */
  uint8_t program_flow;
  int spindle_direction;
  double feed_rate, seek_rate;     /* Millimeters/second */
  double position[3];              /* Where the interpreter considers the tool to be at this point in the code */
  uint8_t tool;
  int16_t spindle_speed;           /* RPM/100 */
  uint8_t plane_axis_0, 
          plane_axis_1, 
          plane_axis_2;            // The axes of the selected plane  
} parser_state_t;
static parser_state_t gc;

#define FAIL(status) gc.status_code = status;

int read_double(char *line,               //  <- string: line of RS274/NGC code being processed
                     int *char_counter,        //  <- pointer to a counter for position on the line 
                     double *double_ptr); //  <- pointer to double to be read                  

int next_statement(char *letter, double *double_ptr, char *line, int *char_counter);


void select_plane(uint8_t axis_0, uint8_t axis_1, uint8_t axis_2) 
{
  gc.plane_axis_0 = axis_0;
  gc.plane_axis_1 = axis_1;
  gc.plane_axis_2 = axis_2;
}

void gc_init() {
  memset(&gc, 0, sizeof(gc));
  gc.feed_rate = settings.default_feed_rate/60;
  gc.seek_rate = settings.default_seek_rate/60;
  select_plane(X_AXIS, Y_AXIS, Z_AXIS);
  gc.absolute_mode = TRUE;
}

inline float to_millimeters(double value) {
  return(gc.inches_mode ? (value * MM_PER_INCH) : value);
}

// Find the angle in radians of deviance from the positive y axis. negative angles to the left of y-axis, 
// positive to the right.
double theta(double x, double y)
{
  double theta = atan(x/fabs(y));
  if (y>0) {
    return(theta);
  } else {
    if (theta>0) 
    {
      return(M_PI-theta);
    } else {
      return(-M_PI-theta);
    }
  }
}

// Executes one line of 0-terminated G-Code. The line is assumed to contain only uppercase
// characters and signed floating point values (no whitespace).
uint8_t gc_execute_line(char *line) {
  int char_counter = 0;  
  char letter;
  double value;
  double unit_converted_value;
  double inverse_feed_rate = -1; // negative inverse_feed_rate means no inverse_feed_rate specified
  int radius_mode = FALSE;
  
  uint8_t absolute_override = FALSE;          /* 1 = absolute motion for this block only {G53} */
  uint8_t next_action = NEXT_ACTION_DEFAULT;  /* The action that will be taken by the parsed line */
  
  double target[3], offset[3];  
  
  double p = 0, r = 0;
  int int_value;

	double homing_dist_to_move = 0;
	double homing_threshold = 0;
	uint16_t homing_max_number_of_times = 0;
	uint8_t spindle_changed = FALSE;
	
  clear_vector(target);
  clear_vector(offset);

  gc.status_code = GCSTATUS_OK;
  
  // Disregard comments and block delete
  if (line[0] == '(') { return(gc.status_code); }
  if (line[0] == '/') { char_counter++; } // ignore block delete  
  
  // If the line starts with an '$' it is a configuration-command
  if (line[0] == '$') { 
    // Parameter lines are on the form '$4=374.3' or '$' to dump current settings
    char_counter = 1;
    if(line[char_counter] == '$') { sp_millInfo(); return(GCSTATUS_OK); }
    if(line[char_counter] == 0) { settings_dump(); return(GCSTATUS_OK); }
    read_double(line, &char_counter, &p);
    if(line[char_counter++] != '=') { return(GCSTATUS_UNSUPPORTED_STATEMENT); }
    read_double(line, &char_counter, &value);
    if(line[char_counter] != 0) { return(GCSTATUS_UNSUPPORTED_STATEMENT); }
    settings_store_setting(p, value);
    return(gc.status_code);
  }
  
  /* We'll handle this as g-code. First: parse all statements */

  // Pass 1: Commands
  while(next_statement(&letter, &value, line, &char_counter)) {
    int_value = trunc(value);
    switch(letter) {
      case 'G':
      switch(int_value) {
        // rapid positioning  
        case 0: gc.motion_mode = MOTION_MODE_SEEK; break;
		// linear interpolation
        case 1: gc.motion_mode = MOTION_MODE_LINEAR; break;
#ifdef __AVR_ATmega328P__   
        // circular/helical interpolation (clockwise)
        case 2: gc.motion_mode = MOTION_MODE_CW_ARC; break; 
        // circular/helical interpolation (counter-clockwise)
        case 3: gc.motion_mode = MOTION_MODE_CCW_ARC; break;
#endif  
		// dwell
        case 4: next_action = NEXT_ACTION_DWELL; break;
        // xy plane selection
        case 17: select_plane(X_AXIS, Y_AXIS, Z_AXIS); break;
        // xz plane selection
        case 18: select_plane(X_AXIS, Z_AXIS, Y_AXIS); break;
        // yz plane selection
        case 19: select_plane(Y_AXIS, Z_AXIS, X_AXIS); break;
        // inch system selection
        case 20: gc.inches_mode = TRUE; break;
        // millimeter system selection
        case 21: gc.inches_mode = FALSE; break;
        // 28: Return to home position (machine zero)
		// 30: Return to secondary home position
		case 28: next_action = NEXT_ACTION_GO_HOME; break;
		// MM_COMMENT - non standard Gcode usage
		case 30: next_action = NEXT_ACTION_MILL_GO_HOME; break;
		// MM_COMMENT - non standard Gcode usage
  		case 31: next_action = NEXT_ACTION_MEASURE_CAP; break;
		// MM_COMMENT - non standard Gcode usage
   	    case 34: next_action = NEXT_ACTION_CUR_POS_IS_ORIGIN; break;
		// MM_COMMENT - non standard Gcode usage
		case 35: next_action = NEXT_ACTION_TURN_OFF_ACCEL; break;
		// MM_COMMENT - non standard Gcode usage	  
		case 36: next_action = NEXT_ACTION_TURN_ON_ACCEL; break;
        // motion in machine coordinate system
        case 53: absolute_override = TRUE; break;
		// Cancel canned cycle
        case 80: gc.motion_mode = MOTION_MODE_CANCEL; break;
        // Absolute Programming
        case 90: gc.absolute_mode = TRUE; break;
        // Incremental programming
        case 91: gc.absolute_mode = FALSE; break;
        // ??
        case 93: gc.inverse_feed_rate_mode = TRUE; break;
        // Feedrate per minute
        case 94: gc.inverse_feed_rate_mode = FALSE; break;
        default: FAIL(GCSTATUS_UNSUPPORTED_STATEMENT);
      }
      break;
      
      case 'M':
      switch(int_value) {
	    // 0: Compulsory stop
	    // 1: Optional stop - stops if operator has pushed the optional stop button
        case 0: case 1: gc.program_flow = PROGRAM_FLOW_PAUSED; break;
	    // 2: End of program
	    // 30: End of program with return to program top
	    // 60: Automatic pallet change (APC) ??
        case 2: case 30: case 60: gc.program_flow = PROGRAM_FLOW_COMPLETED; break;
	    // Spindle on (clockwise rotation)
		  case 3: gc.spindle_direction = 1; spindle_changed = TRUE; break;
	    // Spindle on (counterclockwise rotation)
		 // case 4: gc.spindle_direction = -1; spindle_changed = TRUE; break;
	    // Spindle stop
        case 5: gc.spindle_direction = 0; spindle_changed = TRUE; break;
        default: FAIL(GCSTATUS_UNSUPPORTED_STATEMENT);
      }            
      break;
      case 'T': gc.tool = trunc(value); break;
    }
    if(gc.status_code) { break; }
  }
  
  // If there were any errors parsing this line, we will return right away with the bad news
  if (gc.status_code) { return(gc.status_code); }

  char_counter = 0;
  clear_vector(offset);
  memcpy(target, gc.position, sizeof(target)); // i.e. target = gc.position

  double homing_feed_rate = gc.feed_rate;
	
  // Pass 2: Parameters
  while(next_statement(&letter, &value, line, &char_counter)) {
    int_value = trunc(value);
    unit_converted_value = to_millimeters(value);
    switch(letter) {
	// Feed rate
      case 'F': 
      if (gc.inverse_feed_rate_mode) {
        inverse_feed_rate = unit_converted_value; // seconds per motion for this motion only
      } else if (next_action == NEXT_ACTION_MILL_GO_HOME || next_action == NEXT_ACTION_GO_HOME) {
		homing_feed_rate = unit_converted_value/60;
	  } else {          
        if (gc.motion_mode == MOTION_MODE_SEEK) {
          gc.seek_rate = unit_converted_value/60;
        } else {
          gc.feed_rate = unit_converted_value/60; // millimeters pr second
        }
      }
      break;
	  // I Defines arc size in X axis for G02 or G03 arc commands 
	  // J Defines arc size in Y axis for G02 or G03 arc commands
	  // K Defines arc size in Z axis for G02 or G03 arc commands
      case 'I': case 'J': case 'K': offset[letter-'I'] = unit_converted_value; break;
	  // Serves as parameter adress for various G and M codes.
	  // With G04 defines dwell time.
      case 'P': p = value; break;
	  // Defines size of arc radius or defines retrace height in canned cycles
      case 'R': r = unit_converted_value; radius_mode = TRUE; break;
	  // Speed. Spindle speed or surface speed. 
      case 'S': gc.spindle_speed = value; break;
	  // Absolute or incremental position of X axis
	  // Absolute or incremental position of Y axis
	  // Absolute or incremental position of Z axis
      case 'X': case 'Y': case 'Z':
      if (gc.absolute_mode || absolute_override) {
        target[letter - 'X'] = unit_converted_value;
      } else {
        target[letter - 'X'] += unit_converted_value;
      }
      break;
		case 'A': homing_dist_to_move = unit_converted_value;		break;
			// B - direction (Y)
		case 'B': homing_threshold = value;		break;
			// C - direction (Z)				
		case 'C': homing_max_number_of_times = (uint16_t)trunc(value);		break;	
    }
  }
  
  // If there were any errors parsing this line, we will return right away with the bad news
  if (gc.status_code) { return(gc.status_code); }
    
  // Update spindle state
	if(spindle_changed)
	{
	  if (gc.spindle_direction) {
		  // synchronize (get the mill head to catch up with where we think we are)
		  mc_dwell(0);
		  spindle_run(gc.spindle_direction, gc.spindle_speed);
	  } else {
		  // synchronize (get the mill head to catch up with where we think we are)
		  mc_dwell(0);
		  spindle_stop();
	  }
	}
  
	//TODO_MM - ensure that anything that clears out the position to 0,0,0 in motion_contrl
	// also clears out the gc.position in this file
	
  // Perform any physical actions
  switch (next_action) {		  
    case NEXT_ACTION_GO_HOME: 
		  mc_do_homing_with_params((int)trunc(p), homing_feed_rate, homing_dist_to_move, homing_threshold, homing_max_number_of_times, gc.position);
  		  target[(int)trunc(p)] = 0;
		  break;
	  case NEXT_ACTION_MILL_GO_HOME: 
		  mc_dwell(0);
		  mc_do_mill_homing_with_params(homing_feed_rate, homing_dist_to_move, homing_threshold, homing_max_number_of_times, gc.position);
  		  target[2] = 0;
		  break;		  
	  case NEXT_ACTION_CUR_POS_IS_ORIGIN: 
		  mc_cur_pos_is_origin(trunc(p), gc.position);
		  if( trunc(p) == -1)
		  {
			  target[0] = 0;
			  target[1] = 0;
			  target[2] = 0;
		  } else if(trunc(p) >= 0 && trunc(p) <= 2)
		  {
			  target[(int)trunc(p)] = 0;
		  }
		  break;		  
	  case NEXT_ACTION_TURN_OFF_ACCEL:
		  plan_set_acceleration_manager_enabled(FALSE);
		  break;
	  case NEXT_ACTION_TURN_ON_ACCEL:
		  plan_set_acceleration_manager_enabled(TRUE);
		  break;
	case NEXT_ACTION_DWELL: mc_dwell(trunc(p*1000)); break;
	case NEXT_ACTION_MEASURE_CAP: cc_measure_cap(trunc(p)); break;
    case NEXT_ACTION_DEFAULT: 
    switch (gc.motion_mode) {
      case MOTION_MODE_CANCEL: break;
      case MOTION_MODE_SEEK:
      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], gc.seek_rate, FALSE);
      break;
      case MOTION_MODE_LINEAR:
      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], 
        (gc.inverse_feed_rate_mode) ? inverse_feed_rate : gc.feed_rate, gc.inverse_feed_rate_mode);
      break;
#ifdef __AVR_ATmega328P__
      case MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
      if (radius_mode) {
        /* 
          We need to calculate the center of the circle that has the designated radius and passes
          through both the current position and the target position. This method calculates the following
          set of equations where [x,y] is the vector from current to target position, d == magnitude of 
          that vector, h == hypotenuse of the triangle formed by the radius of the circle, the distance to
          the center of the travel vector. A vector perpendicular to the travel vector [-y,x] is scaled to the 
          length of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2] to form the new point 
          [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the center of our arc.
          
          d^2 == x^2 + y^2
          h^2 == r^2 - (d/2)^2
          i == x/2 - y/d*h
          j == y/2 + x/d*h
          
                                                               O <- [i,j]
                                                            -  |
                                                  r      -     |
                                                      -        |
                                                   -           | h
                                                -              |
                                  [0,0] ->  C -----------------+--------------- T  <- [x,y]
                                            | <------ d/2 ---->|
                    
          C - Current position
          T - Target position
          O - center of circle that pass through both C and T
          d - distance from C to T
          r - designated radius
          h - distance from center of CT to O
          
          Expanding the equations:

          d -> sqrt(x^2 + y^2)
          h -> sqrt(4 * r^2 - x^2 - y^2)/2
          i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2 
          j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
         
          Which can be written:
          
          i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
          j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
          
          Which we for size and speed reasons optimize to:

          h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
          i = (x - (y * h_x2_div_d))/2
          j = (y + (x * h_x2_div_d))/2
          
        */
        
        // Calculate the change in position along each selected axis
        double x = target[gc.plane_axis_0]-gc.position[gc.plane_axis_0];
        double y = target[gc.plane_axis_1]-gc.position[gc.plane_axis_1];
        
        clear_vector(offset);
        double h_x2_div_d = -sqrt(4 * r*r - x*x - y*y)/hypot(x,y); // == -(h * 2 / d)
        // If r is smaller than d, the arc is now traversing the complex plane beyond the reach of any
        // real CNC, and thus - for practical reasons - we will terminate promptly:
        if(isnan(h_x2_div_d)) { FAIL(GCSTATUS_FLOATING_POINT_ERROR); return(gc.status_code); }
        // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
        if (gc.motion_mode == MOTION_MODE_CCW_ARC) { h_x2_div_d = -h_x2_div_d; }
        
        /* The counter clockwise circle lies to the left of the target direction. When offset is positive,
           the left hand circle will be generated - when it is negative the right hand circle is generated.
           
           
                                                         T  <-- Target position
                                                         
                                                         ^ 
              Clockwise circles with this center         |          Clockwise circles with this center will have
              will have > 180 deg of angular travel      |          < 180 deg of angular travel, which is a good thing!
                                               \         |          /   
  center of arc when h_x2_div_d is positive ->  x <----- | -----> x <- center of arc when h_x2_div_d is negative
                                                         |
                                                         |
                                                         
                                                         C  <-- Current position                                 */
                

        // Negative R is g-code-alese for "I want a circle with more than 180 degrees of travel" (go figure!), 
        // even though it is advised against ever generating such circles in a single line of g-code. By 
        // inverting the sign of h_x2_div_d the center of the circles is placed on the opposite side of the line of
        // travel and thus we get the unadvisably long arcs as prescribed.
        if (r < 0) { h_x2_div_d = -h_x2_div_d; }        
        // Complete the operation by calculating the actual center of the arc
        offset[gc.plane_axis_0] = (x-(y*h_x2_div_d))/2;
        offset[gc.plane_axis_1] = (y+(x*h_x2_div_d))/2;
      } 
      
      /*
         This segment sets up an clockwise or counterclockwise arc from the current position to the target position around 
         the center designated by the offset vector. All theta-values measured in radians of deviance from the positive 
         y-axis. 

                            | <- theta == 0
                          * * *                
                        *       *                                               
                      *           *                                             
                      *     O ----T   <- theta_end (e.g. 90 degrees: theta_end == PI/2)                                          
                      *   /                                                     
                        C   <- theta_start (e.g. -145 degrees: theta_start == -PI*(3/4))

      */
            
      // calculate the theta (angle) of the current point
      double theta_start = theta(-offset[gc.plane_axis_0], -offset[gc.plane_axis_1]);
      // calculate the theta (angle) of the target point
      double theta_end = theta(target[gc.plane_axis_0] - offset[gc.plane_axis_0] - gc.position[gc.plane_axis_0], 
         target[gc.plane_axis_1] - offset[gc.plane_axis_1] - gc.position[gc.plane_axis_1]);
      // ensure that the difference is positive so that we have clockwise travel
      if (theta_end < theta_start) { theta_end += 2*M_PI; }
      double angular_travel = theta_end-theta_start;
      // Invert angular motion if the g-code wanted a counterclockwise arc
      if (gc.motion_mode == MOTION_MODE_CCW_ARC) {
        angular_travel = angular_travel-2*M_PI;
      }
      // Find the radius
      double radius = hypot(offset[gc.plane_axis_0], offset[gc.plane_axis_1]);
      // Calculate the motion along the depth axis of the helix
      double depth = target[gc.plane_axis_2]-gc.position[gc.plane_axis_2];
      // Trace the arc
      mc_arc(theta_start, angular_travel, radius, depth, gc.plane_axis_0, gc.plane_axis_1, gc.plane_axis_2, 
        (gc.inverse_feed_rate_mode) ? inverse_feed_rate : gc.feed_rate, gc.inverse_feed_rate_mode,
        gc.position);
      // Finish off with a line to make sure we arrive exactly where we think we are
      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], 
        (gc.inverse_feed_rate_mode) ? inverse_feed_rate : gc.feed_rate, gc.inverse_feed_rate_mode);
      break;
#endif      
    }    
  }
  
  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  memcpy(gc.position, target, sizeof(double)*3); // gc.position[] = target[];
  return(gc.status_code);
}

// Parses the next statement and leaves the counter on the first character following
// the statement. Returns 1 if there was a statements, 0 if end of string was reached
// or there was an error (check state.status_code).
int next_statement(char *letter, double *double_ptr, char *line, int *char_counter) {
  if (line[*char_counter] == 0) {
    return(0); // No more statements
  }
  
  *letter = line[*char_counter];
  if((*letter < 'A') || (*letter > 'Z')) {
    FAIL(GCSTATUS_EXPECTED_COMMAND_LETTER);
    return(0);
  }
  (*char_counter)++;
  if (!read_double(line, char_counter, double_ptr)) {
    return(0);
  };
  return(1);
}

int read_double(char *line,               //!< string: line of RS274/NGC code being processed
                     int *char_counter,   //!< pointer to a counter for position on the line 
                     double *double_ptr)  //!< pointer to double to be read                  
{
  char *start = line + *char_counter;
  char *end;
  
  *double_ptr = strtod(start, &end);
  if(end == start) { 
    FAIL(GCSTATUS_BAD_NUMBER_FORMAT); 
    return(FALSE); 
  };

  *char_counter = end - line;
  return(TRUE);
}

/* 
  Intentionally not supported:

  - Canned cycles
  - Tool radius compensation
  - A,B,C-axes
  - Multiple coordinate systems
  - Evaluation of expressions
  - Variables
  - Multiple home locations
  - Probing
  - Override control

   group 0 = {G10, G28, G30, G92, G92.1, G92.2, G92.3} (Non modal G-codes)
   group 8 = {M7, M8, M9} coolant (special case: M7 and M8 may be active at the same time)
   group 9 = {M48, M49} enable/disable feed and speed override switches
   group 12 = {G54, G55, G56, G57, G58, G59, G59.1, G59.2, G59.3} coordinate system selection
   group 13 = {G61, G61.1, G64} path control mode
*/

