Este repositório foi criado para armazenar o código desenvolvido no trabalho de Robótica Móvel da UFBA, disciplina ministrada pelo Prof. Dr. André Scolari. 


O mapa copiado e utilizado na simulação foi criado por Erick Suzart
- https://github.com/ericksuzart/lar_gazebo

O modelo do pioneer utilizado foi baseado nos seguintes repositórios:
- https://github.com/mario-serna/pioneer_p3dx_model
- https://github.com/SD-Robot-Vision/PioneerModel

Não esqueça de visitar os repositórios citados, dar uma estrela e fazer o clone! São excelentes materiais :)

----

Pacotes necessários:

- python -m pip install -U pip setuptools
- python -m pip install matplotlib
- python -m pip install django
- python -m pip install pandas

----

## Tutorial:

### Passo 1:

- Crie uma pasta na home com um nome qualquer (ex: ufba_ws) e uma pasta src dentro da pasta ufba_ws.
    ```bash
    mkdir -p ~/ufba_ws/src
    ```

### Passo 2:
- Faça o clone deste repositório para dentro da pasta src
  ```bash
  cd ~/ufba_ws/src
  git clone https://github.com/caiobarrosv/pioneer_ufba
  ```

### Passo 3:
- Compile o workspace
    ```bash
    cd ~/ufba_ws
    catkin_make
    ```

### Passo 4
- Não esqueça de referenciar a pasta do seu workspace:
    ```bash
    source ~/ufba_ws/devel/setup.bash
    ```

### Passo 5:
- Use o launch pioneer3dx.launch para abrir o Gazebo e RViz
   ```bash
   roslaunch pioneer_ufba pioneer3dx.launch
   ```

### Passo 6:
- Caso queira utilizar o modelo somente com odometria e sem controlador, use:
    ```bash
    rosrun pioneer_ufba trabalho_1_odometria.py
    ```

- Caso queira utilizar o modelo com odometria + controlador, use:
   ```bash
    rosrun pioneer_ufba trabalho_2_odometria_controle.py
    ```