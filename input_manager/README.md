# input_manager

## Nodes

### input_manager
Odpowiada za publikowanie sterowania z padów do topiców ROSowych. Domyślnie używany z padami LOGITECH F310

#### Published Topics

* **`/input/{JoystickName}_{Id}_{Hostname}`** ([input_msgs/InputMessage])
	
	Sterowanie z pada. Każdy kolejny pad dostaje nowe `{Id}`. `{Hostname}` to nazwa komputera, z którego został uruchomiony Node

## TODO

* namespace publikowanych topiców jako prywatny parametr node'a
* logi ROSowe zamiast printfów

[input_msgs/InputMessage]: https://bitbucket.org/thecontinuum/messages/raw/master/input_msgs/msg/InputMessage.msg