//
// IPR 2010. Pleo Firmware 1.0b. Refer PDK/documentation/
// Author: pushkar
// Native functions
//
#include <Log.inc>
#include <Script.inc>
#include <Sensor.inc>
#include <String.inc>
#include <common/message_type.inc>
#include <pleo/joints.inc>
#include <Joint.inc>
#include <Sound.inc>
#include "commands.inc"

static i;
// static = global variable
// called once on entry
//
public init()
{
  new sensor_name:sn;
  // disable all messages except ours ('script:')
  log_disable(MSG_ALL);
  log_enable(MSG_SCRIPT);


  // all print and printf output is tagged as 'script:'
  for( sn=SENSOR_MAX; sn<=SENSOR_DERIVED_MAX; sn++)
  {
    sensor_disable(sn);
  }
  sensor_enable(SENSOR_TERMINAL);

  printf("MiGIO Pleo Firmware V1.0b\n");
  joint_move_to(12, 90, 128, angle_degrees);
  while(joint_is_moving(12)) sleep;
  joint_move_to(12, -90, 128, angle_degrees);
  while(joint_is_moving(12)) sleep;
//  joint_move_to(12, 0, 128, angle_degrees);

    new joint_name:j;
    new js;
    js=128;
    for (j = JOINT_MIN; j < JOINT_MAX; j++)
    {
      joint_move_to(j, 0, js, angle_degrees);
    }
}

// called on each sensor trigger
public on_sensor(time, sensor_name:sensor, value)
{
    new ch[32];
    new js;
    new sensor_name:sn;

    //printf(".s,%d,%d\n", sensor, value);
    switch (sensor)
    {
      case SENSOR_TERMINAL:
      {
        sensor_read_data(SENSOR_TERMINAL, ch, value);
        //printf(".Received on the terminal: %s\n Characters: %d\n", ch, value);
        if(value == 2) {
          if(ch[0] == 's' && ch[1] == '!') {
            printf("#");
            for( sn=SENSOR_MIN; sn<=SENSOR_MOUTH; sn++) {
              if(sensor_read(sn) > 0) printf("1");
              else printf("0");
            }
            printf("#\n");
          }
          
          if(ch[0] == 'j' && ch[1] == '!') {
			printf("#");
            new joint_name:j;
            for (j = JOINT_MIN; j < JOINT_MAX; j++)
            {
				printf("%d,",joint_get_position(j,angle_degrees));
				if (joint_is_moving(j) > 0) printf("1,");
				else printf("0,");
			}			
			printf("#\n");
          }

          if(ch[0] == 'r' && ch[1] == '!') {
			printf("#");
            new joint_name:j;
            for (j = JOINT_MIN; j < JOINT_MAX; j++)
            {
				printf("%d,",joint_get_min(j,angle_degrees));
				printf("%d,",joint_get_neutral(j,angle_degrees));
				printf("%d,",joint_get_max(j,angle_degrees));
			}			
			printf("#\n");
          }

        }

		if (value == 3)
		{
			if (ch[0] == 'a' && ch[2] == '!')
			{
				if ( ch[1] == '1' )
				{
				  sound_command(cmd_moo);
				}
				if ( ch[1] == '2' )
				{
					sound_command(cmd_snd_grunt);
				}
				if ( ch[1] == '3' )
				{
					sound_command(cmd_howl);
				}
			}
		}

        if(value == 16)
        {
          if(ch[0] == 'm' && ch[15] == '!')
          {
            new joint_name:j;
            js=128;
            for (j = JOINT_MIN; j < JOINT_MAX; j++)
            {
              joint_move_to(j, (ch[j+1]-32-45)*2, js, angle_degrees);
            }
          }
         }
        
        } // end case SENSOR_TERMINAL
      } // end switch
    // returning true will cause the sensor to be reset. if we do not
    // do this, we will continue to be called, unless we do an explicit resetSensor call
    return true;
}

// called when we exit, just before unloading
//
public close()
{
    printf("close\n");
}
