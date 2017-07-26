# Motor Controller - v1

This is version 1. There is always going to be newer versions, and this particular one is a big learning experience

## History

Back in the late 90s I attempted to build a DC motor controller to handle the mobility-scooter motors I had in the garage, destined for my entry for Robot Wars, but it never got any further than trundling to the end of the road. This project attempts to pick up where I left off now that I have been re-invigorated by the return of Robot Wars to the BBC in 2017.

## Concept

I have been inspired, as least partially, by Charles Guan of http://www.etotheipiplusone.net, who was bitten by the motor controller bug a while back, and has been coming up with some great pstuff on his blog.

My board is targetting 100A at about 36v so that I can easily run the 1000/800W 24/36v scooter motors available on eBay and through other outlets - nominally for mini electric scooters and quads, but mainly I'm just aiming to have fun.

## Control Electronics

I'm using an ATmega32u4, the same as in the Arduino Leonardo, and using the Arduino IDE to do my coding. I chose this chip (rather than the venerable ATMega328 that everyone else seems to use on the internet) because I liked the sound of the multi-channel complementary PWM output capabilities - with adjustable deadband. I'm keen to have synchronous rectification so that I don't need to use massive diode, and that I might even finally get my head around how regenerative breaking works

I've got an Intersil HIP4081 to provide the drive current to the FETs. I used this before in my teens, so it seemed a good place to start - in fact I still had the chip in one of my many boxes of bits, so I was able to do a bit of breadboarding to start with.

I've plumped for IRFS7530 in the D2PAK package - not one I've used before, but the specs on these parts looks light-years ahead of what I had used previously: 60v 1.15mOhm typical RDSon, and some stupid theoretical maximum current of 33A (240A package limited). I think it's safe to say I'm unlikely to want to put more than 50A through on (early versions are going to target 2 FETs per leg of the H-Bridge), so that should be good.
