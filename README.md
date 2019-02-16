#Timer555Monostable Library#

The library turns a 555 Timer IC connected in Monostable mode (see 555 Timer datasheet) into a Capacitance and/or Resistance meter. 

The library helps in the design of capacitive sensors which can detect touch or proximity by measuring a variation in an object or sensor Capacitance.

The Capacitance (or self capacitance) of a Sensor (eg a piece of foil) connected to the circuit as C1, changes when in proximity or contact with the human body/object. By reading the capacitance continuously, one is able to determine if an object has been approached (high value of R1) or indeed touched (lower value of R1). The sensor setup requires a medium to high value resistor as R1. The advantage of using a 555 Timer over a single RC circuit (two Pins and one resistor) is the ability to actually measure the (self-)capacitance level (or its variation), in a reliable way without exposing the Arduino board to undue stress from ESB (static electricity). The monostable mode makes it easy to use in any Arduino sketch/project as one retains control over the timing of the measurements (compared to, eg. the Astable mode which oscillates constinuously).  

https://github.com/newEndeavour/Timer555Monostable

any question, please feel free to ask: jerome.p.drouin@gmail.com

