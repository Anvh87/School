#include <Fuzzy.h>            // Embedded fuzzy logic

#define RGB_BRIGHTNESS    64  // Adjust brightness if needed (0-255)
#define TMP36_PIN          2  // Define the pin connected to the TMP36
#define HEATER_PIN    15  // Define the pin connected to the heater
#define READ_RATE       10  // Amount of temperature reads per TMP_UPDATE
#define TMP_UPDATE 5000 // Time in ms to update temperature control




struct Color
{
    unsigned int red;
    unsigned int green;
    unsigned int blue;
};

long setTemp;


// Fuzzy Logic Setup
Fuzzy fuzzy = Fuzzy( );

// FuzzyInput
FuzzySet negativeLarge  = FuzzySet(-30, -30, -20, -10);
FuzzySet negativeSmall  = FuzzySet(-15,  -5,  -5,  -0);
FuzzySet zero           = FuzzySet( -5,   0,   0,   5);
FuzzySet positiveSmall  = FuzzySet(  0,   5,   5,  15);
FuzzySet positiveLarge  = FuzzySet( 10,  20,  30,  30);

// FuzzyOutput
FuzzySet off            = FuzzySet(  0,   0,   0,   0);
FuzzySet low            = FuzzySet(  0,  20,  20,  40);
FuzzySet medium         = FuzzySet( 30,  50,  50,  70);
FuzzySet high           = FuzzySet( 60,  80,  80, 100);
FuzzySet full           = FuzzySet(100, 100, 100, 100);

FuzzyInput  error( 1 ); 
FuzzyOutput pwm  ( 1 );

// Fuzzy rules definition 
// Rule if error nergative large then full
FuzzyRuleAntecedent ifErrorNegativeLarge;
FuzzyRuleConsequent thenPwmfull;
FuzzyRule           fuzzyRule01( 1, &ifErrorNegativeLarge, &thenPwmfull   );

// Rule if error negative small then High
FuzzyRuleAntecedent ifErrorNegativeSmall;
FuzzyRuleConsequent thenPwmHigh;
FuzzyRule           fuzzyRule02( 2, &ifErrorNegativeSmall, &thenPwmHigh   );

// Rule if zero then medium
FuzzyRuleAntecedent ifErrorZero;
FuzzyRuleConsequent thenPwmMedium;
FuzzyRule           fuzzyRule03( 3, &ifErrorZero,          &thenPwmMedium );

// Rule if positive small then low
FuzzyRuleAntecedent ifErrorPositiveSmall;
FuzzyRuleConsequent thenPwmLow;
FuzzyRule           fuzzyRule04( 4, &ifErrorPositiveSmall, &thenPwmLow    );

// Rule if positive large then off
FuzzyRuleAntecedent ifErrorPositiveLarge;
FuzzyRuleConsequent thenPwmOff;
FuzzyRule           fuzzyRule05( 5, &ifErrorPositiveLarge, &thenPwmOff    );


// Define input (Temperature error)
void CreateErrorSet( ) 
{
  error.addFuzzySet( &negativeLarge ); 
  error.addFuzzySet( &negativeSmall );
  error.addFuzzySet( &zero          );
  error.addFuzzySet( &positiveSmall );
  error.addFuzzySet( &positiveLarge );
}

// Define output (PWM duty cycle)
void CreatePwmSet( )
{
  pwm.addFuzzySet( &off    ); 
  pwm.addFuzzySet( &low    ); 
  pwm.addFuzzySet( &medium ); 
  pwm.addFuzzySet( &high   );
  pwm.addFuzzySet( &full   );
}

void ConnectInOut( )
{
  fuzzy.addFuzzyInput ( &error );
  fuzzy.addFuzzyOutput( &pwm   );
}

// Function to map temperature to color
Color getRGBColor( float tempC ) 
{
  Color Result;
  long tempLED = ( tempC   * 10  );
  long tempMin = ( setTemp * 0.5 );        //Makes min threshold 50% of set temp
  long tempMax = ( setTemp * 1.5 );        //Makes max threshold 50% of set temp

  // Blue to Green if lower than setTemp
  if ( tempLED <= setTemp ) 
  {
    Result.red   = constrain( map( tempLED, tempMin, setTemp, 0,              0              ), 0, RGB_BRIGHTNESS );
    Result.green = constrain( map( tempLED, tempMin, setTemp, 0,              RGB_BRIGHTNESS ), 0, RGB_BRIGHTNESS );
    Result.blue  = constrain( map( tempLED, tempMin, setTemp, RGB_BRIGHTNESS, 0              ), 0, RGB_BRIGHTNESS );
  }

  // Green to Red if higher than setTemp
  else
  {
    Result.red   = constrain( map( tempLED, setTemp, tempMax, 0,              RGB_BRIGHTNESS ), 0, RGB_BRIGHTNESS );
    Result.green = constrain( map( tempLED, setTemp, tempMax, RGB_BRIGHTNESS, 0              ), 0, RGB_BRIGHTNESS );
    Result.blue  = constrain( map( tempLED, setTemp, tempMax, 0,              0              ), 0, RGB_BRIGHTNESS );
  }

  return Result;
}

// Function to map temperature to color
Color getRGBColor( float tempC ) 
{
  Color Result;
  long tempLED = ( tempC   * 10  );
  long tempMin = ( setTemp * 0.5 );        //Makes min threshold 50% of set temp
  long tempMax = ( setTemp * 1.5 );        //Makes max threshold 50% of set temp

  // Blue to Green if lower than setTemp
  if ( tempLED <= setTemp ) 
  {
    Result.red   = constrain( map( tempLED, tempMin, setTemp, 0,              0              ), 0, RGB_BRIGHTNESS );
    Result.green = constrain( map( tempLED, tempMin, setTemp, 0,              RGB_BRIGHTNESS ), 0, RGB_BRIGHTNESS );
    Result.blue  = constrain( map( tempLED, tempMin, setTemp, RGB_BRIGHTNESS, 0              ), 0, RGB_BRIGHTNESS );
  }

  // Green to Red if higher than setTemp
  else
  {
    Result.red   = constrain( map( tempLED, setTemp, tempMax, 0,              RGB_BRIGHTNESS ), 0, RGB_BRIGHTNESS );
    Result.green = constrain( map( tempLED, setTemp, tempMax, RGB_BRIGHTNESS, 0              ), 0, RGB_BRIGHTNESS );
    Result.blue  = constrain( map( tempLED, setTemp, tempMax, 0,              0              ), 0, RGB_BRIGHTNESS );
  }

  return Result;
}

void getSsrValue( ) 
{

  static unsigned long lastTime = 0;
  const long interval = TMP_UPDATE;
  static bool state = 0;

  unsigned long now = millis();

  if ( now - lastTime > interval && state == 0) {
    state = 1;
    lastTime = now;
    digitalWrite(HEATER_PIN, HIGH);
  }

  if ( now - lastTime > interval && state == 1) {
    state = 0;
    lastTime = now;
    digitalWrite(HEATER_PIN, LOW);
  }
}


void setup( ) 
{
  Serial.begin( 115200 ); // Initialize serial communication for debugging 

  CreateErrorSet( );
  CreatePwmSet( );
  ConnectInOut( );

  // Rule if error nergative large then full
  ifErrorNegativeLarge.joinSingle( &negativeLarge );
  thenPwmfull.addOutput(           &full          );
  fuzzy.addFuzzyRule(              &fuzzyRule01   );

  // Rule if error negative small then High
  ifErrorNegativeSmall.joinSingle( &negativeSmall );
  thenPwmHigh.addOutput(           &high          );
  fuzzy.addFuzzyRule(              &fuzzyRule02   );

  // Rule if zero then medium
  ifErrorZero.joinSingle(          &zero          );
  thenPwmMedium.addOutput(         &medium        );
  fuzzy.addFuzzyRule(              &fuzzyRule03   );

  // Rule if positive small then low
  ifErrorPositiveSmall.joinSingle( &positiveSmall );
  thenPwmLow.addOutput(            &low           );
  fuzzy.addFuzzyRule(              &fuzzyRule04   );

  // Rule if positive large then off
  ifErrorPositiveLarge.joinSingle( &positiveLarge );
  thenPwmOff.addOutput(            &off           );
  fuzzy.addFuzzyRule(              &fuzzyRule05   );
}


void loopReadTemperature( ) 
{
  //Read new target temperature value from serial if there
  if (Serial.available() > 1)
  {
    float newSetTemp = Serial.parseFloat();
    setTemp          = newSetTemp * 10; 
    Serial.print("Set temperature updated to: ");
    Serial.println( newSetTemp );
    Serial.read( );                                    // Clear the remaining buffer
  }
  // Temperature conversion
  float analogValue = analogRead( TMP36_PIN );         // Read the analog value from the TMP36
  float tempC       = ( analogValue - 500.0 ) / 10.0;  // Calculate temperature in Celsius
  float targetTemp  = ( setTemp / 10 );
  float errorTemp   = tempC - targetTemp;              // Used for fuzzy

  // Changing LED based on temperature
  const auto PixelColor = getRGBColor( tempC );
  
  // Update the LED color based on temperature
  neopixelWrite( RGB_BUILTIN, PixelColor.red, PixelColor.green, PixelColor.blue );

  // Fuzzy Logic Calculation
  fuzzy.setInput( 1, errorTemp );
  fuzzy.fuzzify( );
  float crispPWM = fuzzy.defuzzify( 1 );

  

  Serial.print("Error Temp: ");
  Serial.println( errorTemp );

  Serial.print("PWM VAlue: ");
  Serial.println( crispPWM );

  Serial.print("Target Temperature: ");
  Serial.print( targetTemp );
  Serial.println(" C");

  Serial.print("Current Temperature: ");
  Serial.print( tempC );
  Serial.println(" C");

  delay( UPDATE_TMP / POLL_RATE );
}

void loopSetTemperature( )
{

}

void loop()
{
  loopReadTemperature()
  loopSetTemperature()
}
