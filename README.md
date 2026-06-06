# Trabalho de Programação de Robôs Móveis - Robô que encontra e pega bandeiras

## Pré-requisitos

Certifique-se de que o workspace já foi compilado e que todas as dependências do ROS 2 estão instaladas.

Caso contrário, vá até o diretório raiz do workspace e execute o seguinte comando:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## Execução

Abra **três terminais** e siga os passos abaixo.

### Terminal 1 – Inicialização do Ambiente

Navegue até a pasta do workspace:

```bash
cd ~/seu_workspace/src
```

Carregue o ambiente ROS 2:

```bash
source install/setup.bash
```

Inicie o primeiro nó:

```bash
ros2 launch pega_bandeiras inicia_simulacao.launch.py
```

---

### Terminal 2 – Carregamento do Robô

Navegue até a pasta do workspace:

```bash
cd ~/seu_workspace/src
```

Carregue o ambiente ROS 2:

```bash
source install/setup.bash
```

Execute o lançamento do robô:

```bash
ros2 launch pega_bandeiras carrega_robo.launch.py
```

---

### Terminal 3 – Controle do Robô

Navegue até a pasta do workspace:

```bash
cd ~/seu_workspace/src
```

Carregue o ambiente ROS 2:

```bash
source install/setup.bash
```

Execute o nó de controle:

```bash
ros2 run pega_bandeiras controle_robo.py
```

---

## Resumo Rápido

### Terminal 1

```bash
cd ~/seu_workspace/src
source install/setup.bash
ros2 launch pega_bandeiras inicia_simulacao.launch.py
```

### Terminal 2

```bash
cd ~/seu_workspace/src
source install/setup.bash
ros2 launch pega_bandeiras carrega_robo.launch.py
```

### Terminal 3

```bash
cd ~/seu_workspace/src
source install/setup.bash
ros2 run pega_bandeiras controle_robo.py
```

---

✅ Após iniciar os três terminais, o sistema estará pronto para execução e controle do robô.
