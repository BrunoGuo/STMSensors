# VL53L0X

Funções auxiliares para serem usadas com o sensor [VL53L0X](https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html) da ST Microelectronics. Essa biblioteca foi adaptada da [biblioteca para Arduino](https://github.com/pololu/vl53l0x-arduino) criada pela Pololu, que por sua vez foi inspirada na API oficial da STM, mas não a usa diretamente.

Testado com projetos do STM32CubeMX, usando a biblioteca HAL, e por padrão usa o I2C1, assumindo que ele já está inicializado.

-------

### Changelog

* 09/2018
  * Change ADDRESS_DEFAULT to VL53L0X_DEFAULT_ADDRESS to avoid name collisions
  * Improved in-file documentation
  * Add VL53L0X_I2C_HANDLER, VL53L0X_VCSEL_PRE_RANGE, VL53L0X_VCSEL_FINAL_RANGE constant for easier changing
  * Add VL53L0X_TIMEOUT_RETURN_VALUE

### Funções disponiveis

* `uint8_t vl53l0x_init()`

  Essa função deve ser chamada antes de qualquer tentativa de leitura do sensor, ela inicializa o sensor e retorna 1 em caso de sucesso ou 0, em caso de erro.

* `void vl53l0x_setDevAddress(uint8_t new_addr)`

  Essa função modifica o endereço I2C do dispositivo atual (8 bits), necessário ao utilizar mais de um sensor.

* `void vl53l0x_setCurrentAddress(uint8_t new_addr)`

  Essa função muda o endereço do dispositivo utilizado pelas outras funções (8 bits), não acessa os sensores diretamente.

* `void vl53l0x_startContinuous(uint32_t period_ms)`

  `void vl53l0x_stopContinuous()`

  Essas funções iniciam e param a leitura contínua do sensor atual, respectivamente.

* `uint16_t vl53l0x_getRange()`

  Essa função retorna o último valor lido do sensor atual.

---------

### Modificações

É necessário incluir, em `vl53l0x.h` os arquivos de projeto referentes ao I2C e as definições gerais do HAL, como `stm32f3xx_hal.h`

```c
// vl53l0x.h
13    #ifndef _VL53L0X_H_
14    #define _VL53L0X_H_
15
16    // SPECIFIC INCLUDES HERE
17
18    /*****************************************
```

Também em `vl53l0x.h` pode-se mudar as constantes de acordo com o projeto:
```c
25    #define VL53L0X_I2C_HANDLER hi2c1      /**< I2C handler */
26
27    //! @see setVcselPulsePeriod
28    #define VL53L0X_VCSEL_PRE_RANGE 18
29    #define VL53L0X_VCSEL_FINAL_RANGE 14
```

-----------

### Exemplos

#### Mais de um sensor

```c
void sensors_init() {
    // Desativa os dois sensores (Pino XSHUT)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_Delay(10);

    // Ativa um dos sensores
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(100);
    // Inicializa
    vl53l0x_init();
    // Muda o endereço do sensor
    vl53l0x_setDevAddress(0x54);
    HAL_Delay(10);

    // Ativa o outro sensor
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(100);
    // Inicializa
    vl53l0x_init();
    // Muda o endereço do sensor
    vl53l0x_setDevAddress(0x56);
    HAL_Delay(10);

    HAL_Delay(1);
    // Muda o endereço atual das funções
    vl53l0x_setCurrentAddress(0x54);
    // Inicia as medições do primeiro sensor
    vl53l0x_startContinuous(2);
    HAL_Delay(1);
    // Muda o endereço atual das funções
    vl53l0x_setCurrentAddress(0x56);
    // Inicia as medições do segundo sensor
    vl53l0x_startContinuous(2);
    HAL_Delay(1);
}
```
