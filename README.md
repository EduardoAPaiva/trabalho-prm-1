# 🚀 Como Executar o Projeto

## Pré-requisitos

Certifique-se de que o workspace já foi compilado e que todas as dependências do ROS 2 estão instaladas.

---

## Execução

Abra **três terminais** e siga os passos abaixo.

### Terminal 1 – Inicialização do Ambiente

Navegue até a pasta do workspace:

```bash
cd ~/trabalho-prm/src
```

Carregue o ambiente ROS 2:

```bash
source install/setup.bash
```

Inicie o primeiro nó:

```bash
ros2 launch pega_ba inicia
```

---

### Terminal 2 – Carregamento do Robô

Navegue até a pasta do workspace:

```bash
cd ~/trabalho-prm/src
```

Carregue o ambiente ROS 2:

```bash
source install/setup.bash
```

Execute o lançamento do robô:

```bash
ros2 launch pega_ba carre
```

---

### Terminal 3 – Controle do Robô

Navegue até a pasta do workspace:

```bash
cd ~/trabalho-prm/src
```

Carregue o ambiente ROS 2:

```bash
source install/setup.bash
```

Execute o nó de controle:

```bash
ros2 run pega_ba controle
```

---

## Resumo Rápido

### Terminal 1

```bash
cd ~/trabalho-prm/src
source install/setup.bash
ros2 launch pega_ba inicia
```

### Terminal 2

```bash
cd ~/trabalho-prm/src
source install/setup.bash
ros2 launch pega_ba carre
```

### Terminal 3

```bash
cd ~/trabalho-prm/src
source install/setup.bash
ros2 run pega_ba controle
```

---

✅ Após iniciar os três terminais, o sistema estará pronto para execução e controle do robô.
