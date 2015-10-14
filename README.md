GeoCentinela
============

GeoCentinela Serial API:

'n': Se obtiene la fecha
'j<uint32_t>': Se aplica y guarda el 'begin_time'
'k<uint32_t>': Se aplica y guarda el 'end_time'
'l' : Se obtiene la lista de archivos de la raiz de la SD
'g<char[<=FILENAME_MAX_LENGH]>': Se obtiene el archivo de la raiz de la SD
'r<char[<=FILENAME_MAX_LENGH]>': Se elimina el archivo de la raiz de la SD
'it': Se muestra el hardware ID en ASCII
'ip': Se muestra el hardware ID en binario
'x': Entra en 'xbeee mode', salir con 'x' nuevamente
'so'<uint8_t>: Se aplica y guarda la ganancia indicada
'sg': Se obtiene en binario la configuración
'sp': Se muestra en ASCII la configuración, voltaje de batería, hora, y temperatura
'sm'<uint8_t>: Se aplica y guarda el hardware average
'ss'<uin32_t>: Se aplica y guarda el dt para el sampleo
'st'<uint8_t>: Se aplica y guarda el tipo de alarma
'su'<uint32_t>: Se aplica y guarda el 'begin_time'
'sv'<uint32_t>: Se aplica y guarda el 'end_time'
'sz': Se aplica y guarda para que en 10min más samplee 10min
'sr<uint32_t>': Se syncroniza el reloj en unix epoch
'sw'<uint8_t>: Se aplica la máscara de power
'sy': Se aplica syncGps()
'sx'<uint8_t>: Se aplca y guarda el on/off del GPS
'sq': Se muestra el voltaje de la batería
