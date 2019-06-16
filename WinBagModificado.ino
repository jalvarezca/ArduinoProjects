#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define PIXEL_PIN    14  // Digital IO pin connected to the NeoPixels.
#define PIXEL_COUNT 24  // Number of NeoPixels
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

const int sampleWindow = 250; // Sample window width in mS (250 mS = 4Hz)
unsigned int sample;
unsigned int pixelIndex = 0;

void setup()
{
  Serial.begin(9600);

  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'
  Serial.print("Min");
  Serial.print("\t");

  Serial.print("Max");
  Serial.print("\t");

  Serial.print("Peak");
  Serial.print("\t");

  Serial.print("Volts");
  Serial.print("\t");

  Serial.println("PixelIndex");

}

void loop()
{
  unsigned long startMillis = millis(); // Start of sample window
  int peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  // collect data for 1 second
  while (millis() - startMillis < sampleWindow)
  {
    sample = analogRead(0);
    if (sample < 1024)  //This is the max of the 10-bit ADC so this loop will include all readings
    {
      if (sample > signalMax)
      {
        signalMax = sample;  // save just the max levels
      }
      else if (sample < signalMin)
      {
        signalMin = sample;  // save just the min levels
      }
    }
  }
  peakToPeak = signalMax - signalMin ; // max - min = peak-peak amplitude
  double volts = (peakToPeak * 3.3) / 1024;  // convert to volts



  Serial.print(signalMin);
  Serial.print("\t");

  Serial.print(signalMax);
  Serial.print("\t");

  Serial.print(peakToPeak);
  Serial.print("\t");

  Serial.print(volts);
  Serial.print("\t");

  Serial.println(pixelIndex);

  if (volts >= 0.5) {
    //Encender el siguiente pixel del array
    if (pixelIndex < (PIXEL_COUNT)) {
      if (pixelIndex < 8)
      {
        colorSet(pixelIndex, strip.Color(  0, 255,  0), 100); //Green
      }
      else if (pixelIndex < 16) {
        colorSet(pixelIndex, strip.Color(  0,  0,255), 100); //Blue
      }
      else {
        colorSet(pixelIndex, strip.Color(255,  0,  0), 100); //Red
      }
      ++pixelIndex;
    }
    else if (pixelIndex == PIXEL_COUNT) {
      rainbow(5);
    }
  }
  else
  {
    //Apagar el ultimo pixel que se encendio
    if (pixelIndex >= 0) {
      colorSet(pixelIndex, strip.Color(  0,  0,  0), 100); // Black-Off
      if (pixelIndex > 0) --pixelIndex;
    }
  }
}

void colorSet(uint32_t i,uint32_t color, int wait) {


  strip.setPixelColor(i, color);
  strip.show();
  delay(wait);
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 3 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 3*65536. Adding 256 to firstPixelHue each time
  // means we'll make 3*65536/256 = 768 passes through this outer loop:
  for (long firstPixelHue = 0; firstPixelHue < 3 * 65536; firstPixelHue += 256) {
    for (int i = 0; i < strip.numPixels(); i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}
