# DONT PANIC
... it is a panic button though.  This project is a quick-n-dirty breakout board Lego-style mash up.  The hardware comprises:
- [Arduino Pro Mini 3v3, 8 MHz](https://www.sparkfun.com/products/11114) (Sparkfun)
- [Chronodot v2.1 (DS3231) RTC module](https://www.adafruit.com/product/255) (Adafruit)
- [FONA 800H GSM cellular module](https://www.adafruit.com/product/1946) (Adafruit)
- Odds n ends (various)


## Purpose
Wearable emergency alert buttons (LifeAlert, etc...) have a couple major drawbacks for usability.  
1. They are narrow use case devices (medical emergency only)
2. The battery life isn't great
3. There is NO OPTION except to use the company's call center services.
	
Number 3 is especially difficult if you're a family member, loved one, or acquaintance who wishes to be more directly in the loop 
regarding care for a person aging in place on their own during non life threatening occurences.  Take as an example the internet service goes down, or the phone breaks.  For certain 
users, a simple outreach to a family member or acquaintance might be a more salient step than having to call in the fire brigade.

Sure, it may be an edge case, but it may be entirely reasonable to assume that a septu/oct/non/agenarian may not want to deal with general compute
devices with touch first interfaces and multitude of apps and animations and sound effects and logins with passwords when they have a 
relatively simple need: to connect with someone to alert them to reach out.

## TODO
- Add status and OK led controls with blink patterns
- Fix DS3231 control register write for config alarm on int/sqw pin
- Add routine for acknowledging reply SMS
- Add Parts list
- Add Schematic and board layout
- Test power consumption
- Add instructions for initializing and activating IoT SIM device


## Long Term
- Migrate to 3G+ solution avoid GSM deprecation
- Consider adding e-ink display for readable output
- Look into PTT functionality
- Add voice call option with speaker and mic array