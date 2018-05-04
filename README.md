**MAD-FC-2a** quadrocopter flight controller.

This is a project of a flight controller for quadcopter. The [main source file][cpp] is designed to be descriptive of a multirotor flight controller algorithm. It is a concrete implementation that uses ATmega328P, MPU9255 and nRF24L01+ hardware but they can be easily replaced. The schematic and board layout for the flight controller and the remote board can be found in their respective circuit folders. The project depends on [MAD][madLink] library which is a common library for the flight controller and the remote.
*Not tested, Old project: [FC-alpha2][fcAlpha2].*

Flight Controller
=================
Schematic:
----------
![sch](https://raw.githubusercontent.com/VaSe7u/LiquidMenu/master/doc/Images/logo.png)

Board:
----------
![brd](https://raw.githubusercontent.com/VaSe7u/LiquidMenu/master/doc/Images/logo.png)
![photo](https://raw.githubusercontent.com/VaSe7u/LiquidMenu/master/doc/Images/logo.png)
![photo](https://raw.githubusercontent.com/VaSe7u/LiquidMenu/master/doc/Images/logo.png)

Photos:
----------
![brd](https://raw.githubusercontent.com/VaSe7u/LiquidMenu/master/doc/Images/logo.png)
![photo](https://raw.githubusercontent.com/VaSe7u/LiquidMenu/master/doc/Images/logo.png)
![photo](https://raw.githubusercontent.com/VaSe7u/LiquidMenu/master/doc/Images/logo.png)


Remote
======
Schematic:
----------
![sch](https://raw.githubusercontent.com/VaSe7u/LiquidMenu/master/doc/Images/logo.png)

Board:
----------
![brd](https://raw.githubusercontent.com/VaSe7u/LiquidMenu/master/doc/Images/logo.png)
![photo](https://raw.githubusercontent.com/VaSe7u/LiquidMenu/master/doc/Images/logo.png)
![photo](https://raw.githubusercontent.com/VaSe7u/LiquidMenu/master/doc/Images/logo.png)

Photos:
----------
![brd](https://raw.githubusercontent.com/VaSe7u/LiquidMenu/master/doc/Images/logo.png)
![photo](https://raw.githubusercontent.com/VaSe7u/LiquidMenu/master/doc/Images/logo.png)
![photo](https://raw.githubusercontent.com/VaSe7u/LiquidMenu/master/doc/Images/logo.png)




License
=======
The MIT License (MIT)

Copyright (c) 2016 Vasil Kalchev

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

[madLink]: https://github.com/VaSe7u/MAD
[fcAlpha2]: https://github.com/VaSe7u/FC-alpha2
