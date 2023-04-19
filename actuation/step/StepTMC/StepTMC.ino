//Stepper driver pin definitions
#define PIN_STEP_ENABLE 13
#define PIN_STEP1_DIRECTION 27
#define PIN_STEP1_STEP 33

//Button pin definitions


//Stepper driver variables
int stepsPerSecond = 0;
int currentPosition = 0;

void setup()
{
    //Set both buttons as INPUT_PULLUP so we can detect when they are pressed


    //Condifure stepper driver pins as OUTPUTs
    pinMode(PIN_STEP1_DIRECTION, OUTPUT);
    pinMode(PIN_STEP1_STEP, OUTPUT);
    pinMode(PIN_STEP_ENABLE, OUTPUT);

    //ENABLE pin has to be pulled LOW for TCM2209 used in this example to enable the driver
    digitalWrite(PIN_STEP_ENABLE, LOW);
}

void loop()
{
    //Check if the button is pressed
    
    if (LOW == LOW) {
        //Forward direction
        stepsPerSecond = 1600;
    } else if (HIGH == LOW) {
        //Backward direction
        stepsPerSecond = -1600;    
    } else {
        //If non of the buttons is pressed, set stepsPerSecond to 0
        stepsPerSecond = 0;
    }

    static unsigned long nextChange = 0;
    static uint8_t currentState = LOW;
    if (stepsPerSecond == 0)
    {
        //if speed is 0, set the step pin to LOW to keep current position
        currentState = LOW;
        digitalWrite(PIN_STEP1_STEP, LOW);
    }
    else
    {
        //if stepsPerSecond is not 0, then we need to calculate the next time to change the state of the driver
        if (micros() > nextChange)
        {

            //Generate steps
            if (currentState == LOW)
            {
                currentState = HIGH;
                nextChange = micros() + 30;

                if (stepsPerSecond > 0)
                {
                    currentPosition++;
                }
                else if (stepsPerSecond < 0)
                {
                    currentPosition--;
                }
            }
            else
            {
                currentState = LOW;
                nextChange = micros() + (1000 * abs(1000.0f / stepsPerSecond)) - 30;
            }

            //Set direction based on the sign of stepsPerSecond
            if (stepsPerSecond > 0)
            {
                digitalWrite(PIN_STEP1_DIRECTION, LOW);
            }
            else
            {
                digitalWrite(PIN_STEP1_DIRECTION, HIGH);
            }

            //Write out the step pin
            digitalWrite(PIN_STEP1_STEP, currentState);
        }
    }
}
