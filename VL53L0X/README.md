# VL53L0X

Funções auxiliares para serem usadas com o sensor [VL53L0X](https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html) da ST Microelectronics. Essa biblioteca foi adaptada da [biblioteca para Arduino](https://github.com/pololu/vl53l0x-arduino) criada pela Pololu, que por sua vez foi inspirada na API oficial da STM, mas não a usa diretamente.

Testado com projetos do STM32CubeMX, usando a biblioteca HAL.

---

### Changelog

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

- `uint8_t vl53l0x_i2c_set(I2C_HandleTypeDef* hi2c)`

  Essa função deve ser chamada antes de qualquer outra, ela define qual I2C deve ser utilizado.

- `uint8_t vl53l0x_init()`

  Essa função deve ser chamada antes de qualquer tentativa de leitura do sensor,
  ela inicializa o sensor e retorna 1 em caso de sucesso ou 0, em caso de erro.

- `void vl53l0x_set_dev_address(uint8_t new_addr)`

  Essa função modifica o endereço I2C do dispositivo atual (8 bits), necessário ao utilizar mais de um sensor.

- `void vl53l0x_set_current_address(uint8_t new_addr)`

  Essa função muda o endereço do dispositivo utilizado pelas outras funções (8 bits), não acessa os sensores diretamente.

- `void vl53l0x_start_continuous(uint32_t period_ms)`

  `void vl53l0x_stop_continuous()`

  Essas funções iniciam e param a leitura contínua do sensor atual, respectivamente.

- `uint16_t vl53l0x_get_range()`

  Essa função retorna o último valor lido do sensor atual.

---

### Exemplos

#### Mais de um sensor

```c
void sensors_init() {
    // Define o I2C correto
    vl53l0x_i2c_set(&hi2c2);

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
    vl53l0x_set_dev_address(0x54);
    HAL_Delay(10);

    // Ativa o outro sensor
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(100);
    // Inicializa
    vl53l0x_init();
    // Muda o endereço do sensor
    vl53l0x_set_dev_address(0x56);
    HAL_Delay(10);

    HAL_Delay(1);
    // Muda o endereço atual das funções
    vl53l0x_set_current_address(0x54);
    // Inicia as medições do primeiro sensor
    vl53l0x_start_continuous(2);
    HAL_Delay(1);
    // Muda o endereço atual das funções
    vl53l0x_set_current_address(0x56);
    // Inicia as medições do segundo sensor
    vl53l0x_start_continuous(2);
    HAL_Delay(1);
}
```
