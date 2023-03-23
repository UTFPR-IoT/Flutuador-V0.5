# Flutuador V0.5

Este projeto de código foi desenvolvido para integrar diversos sensores com a placa Gravity e o sensor de temperatura DS18B20. Os sensores utilizados são:

- ADC ADS1X15
- Sensor de temperatura BMP280
- Relógio interno da RTClib da AdaFruit
- Sensores de pH, DO e ORP da categoria ENV-20 da Atlas-Scientific

A integração desses sensores permite a leitura de diversas variáveis ambientais, como temperatura, pressão, pH, DO (oxigênio dissolvido) e ORP (potencial de oxidação-redução). A placa Gravity é utilizada como interface de comunicação entre os sensores e o microcontrolador.

O sensor de temperatura DS18B20 é utilizado para monitorar a temperatura do ambiente em que os outros sensores estão sendo utilizados. Com essa informação, é possível realizar correções nas leituras dos outros sensores que são afetados pela temperatura, como o sensor de pH.

O código foi desenvolvido em linguagem C++ e utiliza bibliotecas específicas para cada sensor. As leituras dos sensores são realizadas a cada intervalo de tempo determinado pelo relógio interno da RTClib.

Com esse projeto, é possível realizar monitoramento ambiental de forma precisa e eficiente, permitindo o controle de variáveis importantes para diversas aplicações, como aquicultura, hidroponia, tratamento de efluentes, entre outras.
