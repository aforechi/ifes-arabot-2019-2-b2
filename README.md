# ifes-arabot-2019-2-a2
Trabalho de C++: Competição de Carros Autônomos em Simuladores de Jogos

1.1 - Equipes
As equipes serão formadas por grupos de:
- projetistas (4º Período) responsáveis por implementar os modelos matemáticos (EDO's) que descrevem a cinemática dos veículos e analisar a precisão e a trajetória dos seus movimentos em ambiente computacional Matlab/Octave tendo como base os conhecimentos adquiridos na disciplina de Cálculo Numérico.
- programadores (2º Período) responsáveis por implementar em C++ o controlador conhecido como Pure Pursuit disponibilizado pelo professor em Matlab/Octave para que o veículo simulado siga uma trajetória de pontos na pista respeitando a velocidade estipulada em cada ponto. A trajetória será gerada (salva) pelos integrantes do grupo ao dirigir o veículo em modo manual e salvar as posições e velocidades correntes a cada metro.

1.2 - Regras ¶
- Dirigir o carro manualmente no ambiente simulado do Microsoft Airsim e salvar num arquivo waypoints.csv as coordenadas x,y por onde o carro passou e a velocidade que estava. Devem ser salvos quatro arquivos com pontos cujas distâncias estejam 1m, 3m e 5m uns dos outros e um original com todas as coordenadas por onde o carro passou.
- Posteriormente, ler o arquivo waypoints.csv e interpolar os pontos (waypoints) para preencher o espaço vazio deixado no intervalo de 1m, 3m e 5m.
- Passar os pontos interpolados para o controlador Pure Pursuit repetir o trajeto de forma autonoma Medir o erro da trajetória executada para a original considerando as diferentes interpolações de 1m, 3m e 5m.
- Rodar o simulador com 10, 20 e 30 FPS, ou seja dt=1/FPS, e medir o erro da trajetória novamente O ranking será formado pelas equipes que obtiverem o menor erro e o menor tempo para completar o percurso pré-estabelecido de waypoints de 3m a 30 FPS.
- Para se classificar as equipes deverão passar por pelo menos 50% dos pontos (waypoints).

2.0 - Modelagem em Octave

- Acesse os notebooks [aqui](https://mybinder.org/v2/gh/aforechi/ifes-num-2018-2/master?filepath=13-simulando-carros-com-edos.ipynb)

3.0 - Implementação em C++

- Determinar o timestep (delta t) da simulação basedo no FPS
- Salvar/Carregar Waypoints (x,y,v) da trajetória
- Obter a posição e velocidade atuais do carro
- Encontre o waypoint mais próximo do carro.
- Interpolar Waypoints: como os waypoints são discretos e afastados uns dos outros (1m, 3m e 5m) e nosso controlador tem um desempenho melhor com um caminho contínuo, enviaremos um subconjunto dos waypoints a uma certa distância do ponto mais próximo do veículo. A interpolação entre cada waypoint fornecerá um caminho de resolução mais fino e o tornará mais "contínuo". Uma interpolação linear simples é usada como método preliminar para resolver esse problema, embora seja melhor abordada com métodos de interpolação melhores (interpolação por spline, por exemplo).
- Enviar valores atuais do estado do veiculo para o controlador: x, y, theta, velocidade
- Obter comandos do controlador aceleração/throttle (entre 0 e 1), direção/steering (entre -1.22rad e 1.22rad), e freio/brake (entre 0 e 1) 
- Enviar comandos para simulação (AirSim)
