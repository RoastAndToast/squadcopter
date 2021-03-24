## Projekta struktūra

main_scene.ttt - Coppelia project

lua - directory with all .lua scripts from Coppelia

models - directory for models

python - directory for .py scripts

include_template.lua - template for Coppelia script to include .lua script from repository /lua directory

## Python skriptu izmantošanas pamācība
		
1. #### Uzstādīt Python v. #.#.#

  	To var izdarīt sekojot šai pamācībai - 
 
2. #### Pārliecināties, ka Coppelia klausās noklusējuma API portu :19997, lai to izdarītu ir nepieciešams atvērt Coppelia instalācijas mapi

  	Noklusējuma mape Windows vidē: `C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu`

  	Noklusējuma mape Linux vidē: `/usr/share/coppeliasim/`

  	Instalācijas mapē atvērt remoteApiConnections.txtfailu un pārliecināties, ka portIndex1_port ir uzstādīts uz 19997.
  	Lai izmaiņas tiktu pārlādētas ir nepieciešams aizvērt restartēt CoppeliaSIM (ja bija ieslēgta)
 
3. #### Ieslēgt izstrādes konsoli CoppeliaSIM vidē, noņemot ķeksi no `Tools -> User Settings -> Hide console window`
 
4. #### Palaist `\squadcopter\python\simpleTest.py` testa failu. Rezultātā, CoppeliaSIM izstrādes konsolē būtu jābūt parādītai peles kursora pozīcijai simulācijas vidē un "Hello CoppeliaSim!" tekstam.
