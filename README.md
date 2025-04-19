# PWM Fan Controller with Internal USB connection

> Controller for PWM-based fans using the internal USB 2.0 header

I have an NVIDIA server GPU which relies on external fans and uses the
`nvidia-smi` cli to report its temperature. In addition, my mainboard
does not allow for fan speed control under
Linux.

As this is my workstation setup and I do not want the fans running at
full speed all the time, I needed a controller to set the fan speed from
software, which easily integrates into my case.

This controller is plugged into an internal USB 2.0 header and offers
the 4-pin PWM header to connect fans.

## License

[![Creative Commons License](https://i.creativecommons.org/l/by-sa/4.0/88x31.png)](http://creativecommons.org/licenses/by-sa/4.0/)

This work is licensed under
a [Creative Commons Attribution-ShareAlike 4.0 International License](http://creativecommons.org/licenses/by-sa/4.0/).

BY: Stefan Haun (mail at tuxathome.de)

See [LICENSE.txt](LICENSE.txt) for details.
