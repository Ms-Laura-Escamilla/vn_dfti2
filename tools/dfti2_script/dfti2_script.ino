// Compilation requires a library called "Vector" installed via ardunio ide libaray manager

# define MOTOR 0
# define SERVO 1

unsigned short int number_of_sensors = 0; // Must be less then 52
bool sensor_type                    [70] = {0};
unsigned short int sensor_pin       [70] = {0};
unsigned short int sensor_state     [70] = {0};
unsigned short int old_sensor_state [70] = {0};
unsigned long int sensor_time       [70] = {0};
unsigned long int rot_start_time    [70] = {0};
unsigned long int rot_period        [70] = {0};
unsigned long int switch_debounce_time = 1000; // In microseconds
unsigned long int stream_rate = 0; // Return stream rate in hz

unsigned long int stream_period = 0;
unsigned long int last_report = micros();

void setup() 
{
  // Get list of sensors, pins, and types as well as data stream rate
  Serial.begin(115200);
  Serial.println("a");
  while(!Serial.available()){}
  number_of_sensors = Serial.parseInt();
  Serial.println(number_of_sensors);
  for(int i = 0;i < number_of_sensors; i++)
  {
    while(!Serial.available()){}
    sensor_type[i] = Serial.parseInt();
    Serial.println(sensor_type[i]);
    while(!Serial.available()){}
    sensor_pin[i]  = Serial.parseInt();
    Serial.println(sensor_pin[i]);
  }
  while(!Serial.available()){}
  stream_rate = Serial.parseInt();
  Serial.println(stream_rate);
  stream_period = round(1000000.0/long(stream_rate));
  Serial.println("b");
  Serial.flush();

  // initialize interupts and pin types
  for(int i = 0;i<number_of_sensors;i++)
  {
    pinMode(sensor_pin[i],INPUT_PULLUP);
  }
}

// I have about 6 ms to play with here
// Serial communication with baud rate of 115200 and .flush takes about 0.6ms So that should be fine.
void loop() 
{
  // Check sensor current state
  for(int i = 0;i<number_of_sensors;i++)
  {
    sensor_state[i] = (sensor_type[i] == SERVO) ? analogRead(sensor_pin[i]) : digitalRead(sensor_pin[i]);
    
    if(sensor_type[i] == MOTOR)
    {
      if(!sensor_state[i] && old_sensor_state[i])
      {
        if(sensor_time[i] <= 0)
        {
          sensor_time[i] = micros();
        }
        else if(micros() - sensor_time[i] > switch_debounce_time)
        {
          old_sensor_state[i] = sensor_state[i];
          if(!sensor_state[i])
          {
            rot_period[i] = sensor_time[i] - rot_start_time[i];
            rot_start_time[i] = sensor_time[i];
          }
          sensor_time[i] = 0;
        }
      }
      else if(sensor_state[i] && !old_sensor_state[i])
      {
        old_sensor_state[i] = sensor_state[i];
      }
      else
      {
        sensor_time[i] = 0;
      }
    }
  }

  // Report result
  unsigned long int time_now = micros();
  if(time_now - last_report > stream_period)
  {
    last_report = time_now;
    Serial.println("c"); // Indicate report is ready
    if(int(Serial.read()))
    {
      for(int i = 0;i<number_of_sensors;i++)
      {
        
        if(sensor_type[i] == SERVO)
        {
          Serial.println(sensor_state[i]);
        }
        else
        {
          if(rot_period[i]!=0)
          {
            Serial.println(1/float(rot_period[i])*1000000.0);
            rot_period[i] = 0;
          }
          else
          {
            Serial.println((0));
          }
        }
      }
    }
    Serial.flush();
  }
}
