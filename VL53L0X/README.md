# VL53L0X

Funções auxiliares para serem usadas com o sensor [VL53L0X](https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html) da ST Microelectronics. Essa biblioteca foi adaptada da [biblioteca para Arduino](https://github.com/pololu/vl53l0x-arduino) criada pela Pololu, que por sua vez foi inspirada na API oficial da STM, mas não a usa diretamente.

Testado com projetos do STM32CubeMX, usando a biblioteca HAL.

---

### Changelog

- 05/2019

  - Add vl53l0x struct, avoids using global variables for i2c and current address.

- 01/2019

  - Remove the need to modify header file, better for submodule use
    - Remove VL53L0X_I2C_HANDLER, VL53L0X_VCSEL_PRE_RANGE, VL53L0X_VCSEL_FINAL_RANGE constants;
    - Add vl53l0x_i2c_set function to set I2C handler, must be called before anything else.
      Can be called again in case different sensors are connected to different I2C ports.
  - Change function names to `snake_case`

- 09/2018

  - Change ADDRESS_DEFAULT to VL53L0X_DEFAULT_ADDRESS to avoid name collisions
  - Improved in-file documentation
  - Add VL53L0X_I2C_HANDLER, VL53L0X_VCSEL_PRE_RANGE, VL53L0X_VCSEL_FINAL_RANGE constant for easier changing
  - Add VL53L0X_TIMEOUT_RETURN_VALUE

### Funções disponiveis

- `uint8_t vl53l0x_init(vl53l0x_t* vl53l0x)`

  Essa função deve ser chamada antes de qualquer tentativa de leitura do sensor,
  ela inicializa o sensor e retorna 1 em caso de sucesso ou 0, em caso de erro.

- `void vl53l0x_xshut_off(vl53l0x_t* vl53l0x)`

  `void vl53l0x_xshut_on(vl53l0x_t* vl53l0x)`

  Essas funções desligam e ligam o sensor através de seu pino xshut.

- `void vl53l0x_set_dev_address(vl53l0x_t* vl53l0x, uint8_t new_addr)`

  Essa função modifica o endereço I2C do dispositivo atual (8 bits), necessário ao utilizar mais de um sensor.

- `void vl53l0x_start_continuous(vl53l0x_t* vl53l0x, uint32_t period_ms)`

  `void vl53l0x_stop_continuous(vl53l0x_t* vl53l0x)`

  Essas funções iniciam e param a leitura contínua do sensor atual, respectivamente.

- `uint16_t vl53l0x_get_range(vl53l0x_t* vl53l0x)`

  Essa função retorna o último valor lido do sensor atual.

---

### Exemplos

#### Mais de um sensor

```c
void sensors_init() {
    vl53l0x_t sensor1 = {
      .hi2c = &hi2c2,
      .addr = VL53L0X_DEFAULT_ADDRESS,
      .xshut_port = GPIOA,
      .xshut_pin = GPIO_PIN_4,
    };

    vl53l0x_t sensor2 = {
      .hi2c = &hi2c2,
      .addr = VL53L0X_DEFAULT_ADDRESS,
      .xshut_port = GPIOA,
      .xshut_pin = GPIO_PIN_5,
    };

    // Desativa os dois sensores (Pino XSHUT)
    vl53l0x_xshut_off(&sensor1);
    vl53l0x_xshut_off(&sensor2);
    HAL_Delay(10);

    // Ativa um dos sensores
    vl53l0x_xshut_on(&sensor1);
    HAL_Delay(100);
    // Inicializa
    vl53l0x_init(&sensor1);
    // Muda o endereço do sensor
    vl53l0x_set_dev_address(&sensor1, 0x54);
    HAL_Delay(10);

    // Ativa o outro sensor
    vl53l0x_xshut_on(&sensor2);
    HAL_Delay(100);
    // Inicializa
    vl53l0x_init(&sensor2);
    // Muda o endereço do sensor
    vl53l0x_set_dev_address(&sensor2, 0x56);
    HAL_Delay(10);

    // Inicia as medições do primeiro sensor
    vl53l0x_start_continuous(&sensor1, 2);
    HAL_Delay(1);

    // Inicia as medições do segundo sensor
    vl53l0x_start_continuous(&sensor2, 2);
    HAL_Delay(1);
}
```
