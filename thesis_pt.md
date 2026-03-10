<img src="media/image1.jpeg" style="width:2.58194in;height:1.53611in" />

AGRADECIMENTOS

Ao meu orientador, **Roberto Finzi**, pela confiança em meu trabalho, pelo comprometimento com a comunicação à distância e, principalmente, pela compreensão e flexibilidade ao permitir que eu trabalhasse em outro estado em paralelo à realização do mestrado.

Ao professor **Leonardo Sanches**, que foi meu co-orientador e professor na primeira etapa do curso, com quem aprendi muitos conceitos utilizados neste trabalho.

Ao **Programa de Pós-Graduação da Faculdade de Engenharia Mecânica (FEMEC)** da **Universidade Federal de Uberlândia (UFU)** que ofereceu estrutura de estudo e pesquisa de qualidade, me fazendo sentir conectado com o universo científico.

Aos colegas do **Laboratório de Aeronaves Autônomas (LAA)**, com destaque ao **Ivan Tarifa**, **Felipe Machini** e **Douglas Costa,** com quem compartilhei experiências práticas relevantes nas áreas de robótica aérea e empreendedorismo.

À **CAPES**, pela bolsa oferecida nos primeiros meses de programa, me possibilitando focar nos estudos acadêmicos.

À psicóloga **Vanda**, que me deu suporte psicológico quando estava ansioso no contexto de iniciação na pós-graduação e na decisão de trabalhar e fazer mestrado em paralelo;

Ao **SENAI CIMATEC**, que, durante o mestrado e em sinal de incentivo à pós-graduação, me deu a oportunidade de trabalhar com projetos de pesquisa e desenvolvimento em robótica de forma remunerada, me colocando em contato com profissionais competentes e tecnologia de ponta, fazendo-me crescer profissionalmente e academicamente.

Ao amigo **Murilo Mendonça**, que solicitamente revisou meu trabalho na reta final.

Aos meus pais **Antônio** e **Cleide**, que sempre me apoiaram incondicionalmente em todos os momentos desafiantes da vida que, a propósito, foram muitos nos últimos dois anos.

À minha esposa **Camilla**, com quem tenho vivido junto há quase um ano e cuja convivência saudável me impulsionou a finalizar o trabalho dentro do prazo.

A **Deus**, que, às vezes por linhas tortas, mas sempre certo, vem promovendo circunstâncias de muito aprendizado, conquistas e alegrias em minha vida.

Em memória do meu irmão **Filipe**, ao qual remete a lembrança de uma pessoa enérgica e determinada, características estas me demandadas para concluir este trabalho.

Araújo, M. A. CONTROLE POR MODOS DESLIZANTES DE QUADCÓPTERO COM CARGA SUSPENSA POR CABO PARA TRAJETÓRIAS BASEADAS NA PROPRIEDADE DE PLANICIDADE DIFERENCIAL DO SISTEMA E INPUT SHAPING. 100 f. Dissertação de Mestrado, Universidade Federal de Uberlândia, Uberlândia.

**Resumo**

No contexto de alta de demanda por aeronaves autônomas no serviço de transporte de carga, este trabalho apresenta uma solução de controle e geração de trajetória para o sistema quadcóptero com carga suspensa por cabo no intuito de controlar a posição da aeronave ao mesmo tempo que reduzir o balanço da carga. Primeiramente, desenvolve-se um modelo dinâmico do sistema com os métodos de Newon-Euler e Euler-Lagrange e separa-o em duas partes: um subsistema totalmente atuado associado à altitude e ângulo de guinada e um subsistema sub-atuado com as outras variáveis do sistema. Cada subsistema é controlador por um controlador por modos deslizantes que é demonstrado ser estável no sentido de Lyapunov na tarefa de conduzir o sistema às superfícies deslizantes e mantê-lo nesta condição. Também é demonstrado pelo critério de estabilidade Routh-Hurwitz que as variáveis deslizantes associadas ao subsistema sub-atuado são localmente estáveis, obtendo-se regras de definição dos parâmetros de controle que facilitam o processo de ajuste. Finalmente, propõe-se uma nova configuração de geração de trajetória para conter o balanço da carga. A estratégia consiste em construir uma trajetória polinomial ponto a ponto para a carga às quais se aplica *input shaping* e então calcula-se o estado desejado da aeronave fazendo-se uso da propriedade de planicidade diferencial do sistema. Verifica-se por meio de simulação que o controlador é eficaz no controle da aeronave e que o gerador de trajetórias cumpre bem o papel de redução da oscilação da carga, em especial para manobras de baixa e média agressividade. Alternativamente, verificou-se que a aplicação de *input shaping* diretamente em trajetórias definidas para a aeronave também atenua o balanço da carga mesmo para manobras agressivas em que a nova solução proposta apresentou resultados insatisfatórios.

Araujo, M. A. **Sliding Mode Control of a Quadrotor with a Suspended Load for Trajectories based on the Differential Flatness Property of the System and Input Shaping.** 100 p. M. Sc. Dissertation, Federal University of Uberlandia, Uberlandia.

**Abstract**

Given the high demand for autonomous aircrafts in cargo transport applications, this work presents a motion control and trajectory generation solution for the problem of controlling the state of a quadrotor carrying a cabled-suspended payload while keeping the load swing stable. First, it develops the dynamic model of the system using the Newton-Euler and Euler-Lagrange methods and divides it into two parts: a fully actuated subsystem associated with the robot altitude and yaw angle, and an underactuated subsystem associated with the other state variables of the quadrotor. Each subsystem is controlled by a sliding mode controller which is proved to be stable in Lyapunov's sense for the task of driving the system to the sliding surfaces and staying on them. It is demonstrated by the Routh-Hurwitz stability criterion that the sliding surfaces associated with the underactuated subsystem are locally stable, finding constraint rules for the control parameters that helps the tuning process. Finally, a new trajectory generation structure is proposed to suppress the load balance. The strategy consists on build a point-to-point piecewise polynomial trajectory for the load, apply input shaping on it and compute the desired state of the aircraft by making use of the differential flatness property of the system. It is verified by simulation that the proposed solution effectively controls the aircraft position and greatly reduces the load swing, especially for low and mid-aggressive maneuvers. Alternatively, this work also tests the application of input shaping directly on the quadrotor trajectories, verifying it attenuates the load balance even for aggressive maneuvers that the proposed solution presented unsatisfactory results.

LISTA DE SÍMBOLOS

| $$\alpha_{RMS}$$                                                                                                                   | Valor eficaz do ângulo do cabo em relação à vertical durante o tempo de acomodação para amostras de uma simulação.                                                   |
|------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| $$\beta_{RMS}$$                                                                                                                    | Valor eficaz do ângulo entre o eixo vertical inercial (${\overrightarrow{e}}_{z}$) e o não inercial (${\overrightarrow{e}}_{z}^{b}$) para amostras de uma simulação. |
| $$\mathbf{B}\left( \overrightarrow{q} \right)$$                                                                                    | Matriz de entrada do modelo dinâmico translacional do sistema.                                                                                                       |
| $$c_{x},\ c_{y},c_{z}$$                                                                                                            | Coeficiente de arrasto translacional linear da aeronave nos eixos x, y e z do referencial do corpo respectivamente.                                                  |
| $$c_{d}$$                                                                                                                          | Coeficiente de arrasto translacional linear da carga.                                                                                                                |
| $$\mathbf{C}\left( \overrightarrow{q},\dot{\overrightarrow{q}} \right)$$                                                           | Matriz de forças centrífuga e de *Coriolis* do modelo dinâmico translacional do sistema.                                                                             |
| $$d$$                                                                                                                              | Comprimento de cada braço do quadcóptero.                                                                                                                            |
| $$d_{x},d_{y},d_{x}$$                                                                                                              | Distúrbio sobre as acelerações de translação do quadcóptero.                                                                                                         |
| $$d_{\phi},d_{\theta},d_{\psi}$$                                                                                                   | Distúrbio sobre as acelerações dos ângulos de Euler que descrevem a orientação da aeronave.                                                                          |
| $$d_{\phi_{L}},\ d_{\theta_{L}}$$                                                                                                  | Distúrbio sobre as acelerações dos ângulos $\phi_{L}$ e $\theta_{L}$, que descrevem a orientação do cabo                                                             |
| $${\overrightarrow{D}}_{F} = \left\lbrack D_{F}^{x},D_{F}^{y},D_{F}^{z} \right\rbrack^{T}$$                                        | Distúrbio de força sobre o sistema no referencial inercial.                                                                                                          |
| $${\overrightarrow{D}}_{\tau} = \left\lbrack D_{\tau}^{x},D_{\tau}^{y},D_{\tau}^{z} \right\rbrack^{T}$$                            | Distúrbio de momento sobre a aeronave no referencial não inercial.                                                                                                   |
| $$\Sigma^{i} = \left\lbrack {\overrightarrow{e}}_{x},{\overrightarrow{e}}_{y},{\overrightarrow{e}}_{z} \right\rbrack$$             | Sistema de coordenadas inercial.                                                                                                                                     |
| $$\Sigma^{b} = \left\lbrack {\overrightarrow{e}}_{x}^{b},{\overrightarrow{e}}_{y}^{b},{\overrightarrow{e}}_{z}^{b} \right\rbrack$$ | Sistema de coordenadas não inercial localizado no centro de massa da aeronave                                                                                        |
| $$\Sigma^{c} = \left\lbrack {\overrightarrow{e}}_{x}^{c},{\overrightarrow{e}}_{y}^{c},{\overrightarrow{e}}_{z}^{c} \right\rbrack$$ | Sistema de coordenadas intermediário resultante da rotação de ângulo $\psi$ em torno do eixo z do referencial inercial.                                              |
| $$\Sigma^{d} = \left\lbrack {\overrightarrow{e}}_{x}^{d},{\overrightarrow{e}}_{y}^{d},{\overrightarrow{e}}_{z}^{d} \right\rbrack$$ | Sistema de coordenadas intermediário resultante da rotação de ângulo $\theta$ em torno do eixo y do referencial $\Sigma^{c}$.                                        |
| $${\overrightarrow{F}}_{b} = \left\lbrack 0,0,F_{z}^{b} \right\rbrack^{T}$$                                                        | Força de propulsão resultante na aeronave no referencial do corpo.                                                                                                   |
| $$F_{z}^{b} = \sum_{i = 1}^{4}F_{i}$$                                                                                              | Força de propulsão ao longo de ${{\overrightarrow{e}}_{b}}_{z}$, sendo $F_{i}$ a força de propulsão gerada por cada rotor.                                           |
| $${\overrightarrow{F}}_{d}$$                                                                                                       | Força de arrasto linear translacional no sistema.                                                                                                                    |
| $${\overline{f}}_{\omega}$$                                                                                                        | Frequência media do módulo da frequência angular $\overrightarrow{\omega}$ calculado sobre amostras de uma simulação.                                                |
| $$g$$                                                                                                                              | Aceleração gravitacional.                                                                                                                                            |
| $$\mathbf{G}\left( \overrightarrow{q} \right)$$                                                                                    | Vetor associado ao esforço gravitacional na dinâmica translacional da aeronave no modelo matricial obtido pela formulação de *Lagrange*.                             |
| $${\overrightarrow{h}}_{\omega},\ {\overrightarrow{h}}_{\alpha}$$                                                                  | Resultado do produto vetorial da velocidade e aceleração angulares no referencial do corpo com ${\overrightarrow{e}}_{z}^{b}$, respectivamente.                      |
| $$\eta_{z},\eta_{\psi},\eta_{1},\eta_{2}$$                                                                                         | Parâmetros de controle que garante robustez contra distúrbios limitados                                                                                              |
| $$\overrightarrow{\eta} = \lbrack\phi,\theta,\psi\rbrack^{T}$$                                                                     | Ângulos que definem da orientação do *drone* segundo a notação de Euler: ângulos de rolagem, arfagem e guinada respectivamente                                       |
| $${\overrightarrow{\eta}}_{L} = \left\lbrack \phi_{L},\theta_{L} \right\rbrack^{T}$$                                               | Ângulos que definem a orientação do cabo obtidos por uma rotação no eixo x ($\phi_{L}$) seguida e outra no eixo y                                                    |
| $$\mathbf{I} = diag\ \left( I_{x},\ I_{y},I_{z} \right)$$                                                                          | Momento de inércia do quadcóptero.                                                                                                                                   |
| $$k_{t},k_{m}$$                                                                                                                    | Constante de propulsão e de arrasto das hélices                                                                                                                      |
| $$\kappa_{z},\kappa_{\psi},\kappa_{1},\kappa_{2}$$                                                                                 | Parâmetros de controle que multiplicam linearmente as variáveis deslizantes                                                                                          |
| $$l$$                                                                                                                              | Comprimento do cabo.                                                                                                                                                 |
| $$L$$                                                                                                                              | Lagrangiano associado ao modelo dinâmico translacional do sistema                                                                                                    |
| $$\lambda_{z},\lambda_{\psi},\ \lambda_{i = 1,2,\ldots,8}$$                                                                        | Parâmetros de controle presentes na definição das variáveis deslizantes                                                                                              |
| $$M,\ m$$                                                                                                                          | Massa da aeronave e da carga respectivamente                                                                                                                         |
| $$\mathbf{M}\left( \overrightarrow{q} \right)$$                                                                                    | Matriz de inércia do modelo dinâmico de translação do sistema.                                                                                                       |
| $$\overrightarrow{p}$$                                                                                                             | Vetor unitário que parte do centro de massa do *drone* até o centro da carga, descrevendo a orientação do cabo                                                       |
| $$\mathbf{P}\left( \overrightarrow{q} \right)$$                                                                                    | Matriz de forças de arrasto translacional linear no modelo dinâmico matricial do sistema obtido pela formulação de Euler-Lagrange.                                   |
| $$\overrightarrow{q} = \left\lbrack x,y,z,\phi_{L},\theta_{L} \right\rbrack$$                                                      | Coordenadas generalizada do modelo dinâmico de translação do sistema.                                                                                                |
| $$\overrightarrow{r} = \lbrack x,y,z\rbrack$$                                                                                      | Posição do *drone* no referencial inercial                                                                                                                           |
| $${\overrightarrow{r}}_{L} = \left\lbrack x_{L},y_{L},z_{L} \right\rbrack$$                                                        | Posição da carga no referencial inercial                                                                                                                             |
| $$s_{1},s_{2},s_{3},s_{4}$$                                                                                                        | Variáveis deslizantes.                                                                                                                                               |
| $$\overrightarrow{T} = T\ \overrightarrow{p}$$                                                                                     | Força de tração no cabo com módulo $T$ ao longo de $\overrightarrow{p}$.                                                                                             |
| $${\overrightarrow{\tau}}_{b} = \left\lbrack \tau_{x_{b}},\ {\tau_{y}}_{b},\ {\tau_{z}}_{b} \right\rbrack^{T}$$                    | Momento resultante em torno do centro de massa da aeronave produzido pela força de propulsão dos rotores e arrasto nas hélices                                       |
| $$\overrightarrow{u} = \left\lbrack u_{1},u_{2},u_{3},u_{4} \right\rbrack^{T}$$                                                    | Sinal de controle, correspondendo aos esforços de entrada do sistema $\left\lbrack F_{b},{\tau_{b}}_{x},{\tau_{b}}_{y},{\tau_{b}}_{z} \right\rbrack^{T}$             |
| $$\overrightarrow{v} = \lbrack u,v,w\rbrack$$                                                                                      | Velocidade de translação do *drone* no referencial não inercial                                                                                                      |
| $$V(x)$$                                                                                                                           | Função de Lyapunov em relação à variável $x$.                                                                                                                        |
| $${\widetilde{x}}_{b},{\widetilde{y}}_{b}$$                                                                                        | Componentes $x$ e $y$ do erro de posição da aeronave em relação referencial não inercial projetado no plano $xy$                                                     |
| $$\omega_{n}$$                                                                                                                     | Frequência natural.                                                                                                                                                  |
| $$\overrightarrow{\omega} = \lbrack p,q,r\rbrack^{T}$$                                                                             | Velocidade angular do drone em relação ao referencial não inercial                                                                                                   |
| $$\overrightarrow{\Omega} = \left\lbrack \dot{\phi},\dot{\theta},\dot{\psi} \right\rbrack^{T}$$                                    | Taxa de variação dos ângulos Euler ($\dot{\overrightarrow{\eta}}$)                                                                                                   |
| $$\zeta$$                                                                                                                          | Coeficiente de amortecimento.                                                                                                                                        |

SUMÁRIO

1.  [INTRODUÇÃO [11](#_Toc24304534)](#_Toc24304534)

[1.1 Motivação e Aplicações [11](#motivação-e-aplicações)](#motivação-e-aplicações)

[1.2 Trabalhos Relacionados [14](#trabalhos-relacionados)](#trabalhos-relacionados)

[1.2.1 Controle em malha aberta [15](#controle-em-malha-aberta)](#controle-em-malha-aberta)

[1.2.2 Controle em Malha Fechada [16](#controle-em-malha-fechada)](#controle-em-malha-fechada)

[1.3 Objetivo e Contribuições [16](#objetivo-e-contribuições)](#objetivo-e-contribuições)

[1.4 Estrutura do Documento [18](#estrutura-do-documento)](#estrutura-do-documento)

2.  [MODELO DINÂMICO [19](#modelo-dinâmico)](#modelo-dinâmico)

[2.1 Drone sem Carga [19](#drone-sem-carga)](#drone-sem-carga)

[2.2 *Drone* com Carga Suspensa por Cabo [23](#drone-com-carga-suspensa-por-cabo)](#drone-com-carga-suspensa-por-cabo)

3.  [CONTROLE [30](#controle)](#controle)

[3.1 Introdução [30](#introdução-1)](#introdução-1)

[3.1.1 Características de Atuação [30](#características-de-atuação)](#características-de-atuação)

[3.1.2 Controle por Modos Deslizantes (CMD) [32](#controle-por-modos-deslizantes-cmd)](#controle-por-modos-deslizantes-cmd)

[3.2 Estratégia de Controle [35](#estratégia-de-controle)](#estratégia-de-controle)

[3.2.1 CMD do Subsistema Totalmente Atuado ($z,\psi$) [36](#cmd-do-subsistema-totalmente-atuado-zpsi)](#cmd-do-subsistema-totalmente-atuado-zpsi)

[3.2.2 CMD do Subsistema Sub-atuado [40](#cmd-do-subsistema-sub-atuado)](#cmd-do-subsistema-sub-atuado)

[3.2.3 Resumo [49](#resumo)](#resumo)

[3.3 Simulação [50](#simulação)](#simulação)

[3.3.1 Resposta do sistema para entrada degrau unitário [51](#resposta-do-sistema-para-entrada-degrau-unitário)](#resposta-do-sistema-para-entrada-degrau-unitário)

[3.3.2 Avaliação da condição de estabilidade na superfície deslizante [55](#avaliação-da-condição-de-estabilidade-na-superfície-deslizante)](#avaliação-da-condição-de-estabilidade-na-superfície-deslizante)

4.  [GERAÇÃO DE TRAJETÓRIAS [58](#geração-de-trajetórias)](#geração-de-trajetórias)

[4.1 Geração de Trajetória com Base na Planicidade Diferencial do Sistema [58](#geração-de-trajetória-com-base-na-planicidade-diferencial-do-sistema)](#geração-de-trajetória-com-base-na-planicidade-diferencial-do-sistema)

[4.1.1 Planicidade Diferencial do Sistema [59](#planicidade-diferencial-do-sistema)](#planicidade-diferencial-do-sistema)

[4.1.2 Determinação das Variáveis do Sistema [60](#determinação-das-variáveis-do-sistema)](#determinação-das-variáveis-do-sistema)

[4.1.3 Definição de Trajetórias para a Carga [64](#definição-de-trajetórias-para-a-carga)](#definição-de-trajetórias-para-a-carga)

[4.2 Input Shaping [65](#input-shaping)](#input-shaping)

[4.2.1 Fundamentação Teórica [65](#fundamentação-teórica)](#fundamentação-teórica)

[4.2.2 Input shaping aplicado ao problema [67](#input-shaping-aplicado-ao-problema)](#input-shaping-aplicado-ao-problema)

[4.3 Trajetórias Baseadas na Planicidade Diferencial do Sistema com *Input Shaping* [69](#trajetórias-baseadas-na-planicidade-diferencial-do-sistema-com-input-shaping)](#trajetórias-baseadas-na-planicidade-diferencial-do-sistema-com-input-shaping)

5.  [CONTROLADOR COM GERADOR DE TRAJETÓRIA [74](#controlador-com-gerador-de-trajetória)](#controlador-com-gerador-de-trajetória)

[5.1 Estrutura da Análise [74](#estrutura-da-análise)](#estrutura-da-análise)

[5.2 Análise dos Resultados [77](#análise-dos-resultados)](#análise-dos-resultados)

6.  [CONCLUSÕES [87](#conclusões)](#conclusões)

[REFERÊNCIAS BIBLIOGRÁFICAS [89](#referências-bibliográficas)](#referências-bibliográficas)[APÊNDICE I – TRANSFORMAÇÕES CINEMÁTICAS [95](#apêndice-i-transformações-cinemáticas)](#apêndice-i-transformações-cinemáticas)[Ângulos de Euler [95](#ângulos-de-euler)](#ângulos-de-euler)

[Transformação da Velocidade Angular [96](#transformação-da-velocidade-angular)](#transformação-da-velocidade-angular)

[APÊNDICE II – INTERPOLAÇÃO POLINOMIAL POR PARTES [98](#apêndice-ii-interpolação-polinomial-por-partes)](#apêndice-ii-interpolação-polinomial-por-partes)

1.  

# INTRODUÇÃO

O desenvolvimento de veículos aéreos não tripulados (VANTs), também chamados de *drones*, tem tido muito destaque em pesquisas no meio acadêmico e empresarial nos últimos anos. Comparado aos veículos aéreos tripulados, os VANTs eliminam o risco ao piloto, promovem redução significativa de tamanho e custo, além de possuírem uma vasta gama de aplicações. Atualmente, *drones* são muito utilizados para realizar fotografias aérea, sensoriamento remoto, inspeção de linhas de transmissão, pulverização de plantações e até entregas aéreas por exemplo.

Porém, pesquisas mais recentes têm se voltado a realizar tarefas cada vez mais complexas. Segundo Ding et al. (2018), um dos ramos de pesquisa com *drones* que tem crescido significativamente nos últimos anos é o da manipulação aérea, em que a aeronave interage fisicamente com o ambiente. Esta área envolve diversas aplicações, como transporte de carga, construção, inspeção por contato e operação remota. Além de apresentar grande potencial de aplicações, este problema atrai o interesse de pesquisadores pelo grande desafio de engenharia envolvido, em especial nas áreas de modelagem e controle.

Neste contexto, em alinhamento com estado da arte em controle e robótica aérea, o presente trabalho propõe realizar o controle de aeronaves do tipo multi-rotor no transporte de carga suspensa por cabo.

## Motivação e Aplicações

Atualmente, helicópteros equipados com elementos de içamento de cargas são utilizados em muitas aplicações, como para transportar madeira em regiões de extração de difícil acesso terrestre (Figura 1.1-a), para coletar e lançar água em missões de combate a incêndios (Figura 1.1-b) e para manipular grandes estruturas (Figura 1.1-c), como torres de transmissão (VARGAS MORENO, 2017; PDG Aviation Services, 2018).

<img src="media/image2.png" style="width:5.90551in;height:2.66885in" />

Figura 1.1 - Aplicações de transporte de carga suspensa por helicóptero. (a) transporte de árvores (PERKOWSKI, 2015), (b) combate a incêndios (BOB, \[s.d.\]) e (c) transporte de torres de transmissão (SHEPHERD; JARVIS; HUNT, 2014).

Porém, essas operações são de alto risco ao piloto e demanda capacitação especializada, fatores que motivam a investigação de VANTs para estas aplicações. Eles podem realizar este tipo de missão com mais agilidade, precisão e segurança, podendo operar de forma inteligente sem a intervenção humana.

A utilização de aeronaves autônomas também viabiliza o transporte aéreo de cargas menores (Figura 2-a), podendo ser explorado para entregas a domicílio, envio de suprimentos para regiões de difícil acesso em situações de desastres (FAUST et al., 2017), resgate de pessoas e animais em situações de risco, lançamento de robôs de exploração em áreas remotas por exemplo.

<img src="media/image3.emf" style="width:5.90551in;height:2.71038in" />

Figura 1.2 - Aplicações específicas de *drone* com carga suspensa por cabo. (a) transporte de suprimentos (FAUST et al., 2017), (b) detecção de minas (BISGAARD, 2008) e (c) coleta de amostras de água (ORE et al., 2015).

Além de servir para transporte de carga em geral, este sistema pode ter utilidades específicas. Por exemplo, Bisgaard (2008) desenvolve uma solução de controle completa de um helicóptero na manipulação de equipamento de localização de minas (Figura 2-b). A aeronave mantém o aparato estabilizado próximo ao chão, eliminando o risco de explosão e aumentando a velocidade da operação. *Drones* com cabos são usados para coletar amostras de água para análise em rios e lagos, como mostra a Figura 1-c). Assim, elimina-se a necessidade de mobilização de equipes com barcos, reduzindo custo, aumentando a agilidade e segurança da operação (ORE et al., 2015).

Em comparação a sistemas em que a carga é rigidamente conectada ao *drone*, a utilização de cabos confere maior agilidade à aeronave, visto que não altera a sua inércia de rotação e permite carregar objetos de maiores dimensões e formatos diversos. Por outro lado, a carga suspensa agrega mais complexidade ao sistema, apresentando movimento que não é diretamente controlado e que é sensível a distúrbios externos, provocando perturbações significativas na dinâmica da aeronave. Dessa forma, faz-se necessário o desenvolvimento de controladores especializados que consideram este acoplamento dinâmico para produzir movimentos desejados para o conjunto drone e carga.

## Trabalhos Relacionados

A comunidade acadêmica tem demonstrado interesse significativo pelo tema de içamento de cargas na última década. Com base na abordagem feita nos estudos de controle de pontes rolantes apresentado por Qian e Yi (2015) e Ramli et al. p. 20 (2017), identificou-se que as pesquisas de controle de *drones* com carga suspensa por cabo podem ser divididas em duas categorias: malha aberta e malha fechada. A Tabela 1.1 apresenta um resumo das principais técnicas utilizadas e os principais trabalhos de referência.

Tabela 1.1 - Técnicas de controle aplicadas ao sistema drone com carga suspensa por cabo.

<table>
<colgroup>
<col style="width: 12%" />
<col style="width: 45%" />
<col style="width: 41%" />
</colgroup>
<thead>
<tr class="header">
<th><strong>Categoria</strong></th>
<th><strong>Estratégia</strong></th>
<th><strong>Referências</strong></th>
</tr>
</thead>
<tbody>
<tr class="odd">
<td rowspan="4"><strong>Malha Aberta</strong></td>
<td><em>Input Shaping</em></td>
<td>(BISGAARD; COUR-HARBO; BENDTSEN, 2008; KLAUSEN; FOSSEN; JOHANSEN, 2015, 2017)</td>
</tr>
<tr class="even">
<td>Otimização de trajetória - programação dinâmica</td>
<td>(PALUNKO; FIERRO; CRUZ, 2012)</td>
</tr>
<tr class="odd">
<td>Geração de trajetória por agente inteligente obtido com aprendizado por reforço</td>
<td>(FAUST et al., 2013, 2017)</td>
</tr>
<tr class="even">
<td>Geração de trajetória com base na propriedade de sistema diferencialmente plano</td>
<td>(SREENATH; LEE; KUMAR, 2013; SREENATH; MICHAEL; KUMAR, 2013)</td>
</tr>
<tr class="odd">
<td rowspan="7"><p><strong>Malha</strong></p>
<p><strong>Fechada</strong></p></td>
<td>Backstepping</td>
<td>(KLAUSEN; FOSSEN; JOHANSEN, 2015, 2017)</td>
</tr>
<tr class="even">
<td>Controle por modos deslizantes (<em>Sliding Mode Control – SMC</em>)</td>
<td>(KUI et al., 2017; ZHOU et al., 2016)</td>
</tr>
<tr class="odd">
<td>Controle baseado em passividade</td>
<td>(GUERRERO et al., 2015a, 2015b; GUERRERO-SÁNCHEZ et al., 2017a)</td>
</tr>
<tr class="even">
<td>Controle Geométrico</td>
<td>(GOODARZI; LEE; LEE, 2014; KOTARU; WU; SREENATH, 2017; SREENATH; LEE; KUMAR, 2013; SREENATH; MICHAEL; KUMAR, 2013)</td>
</tr>
<tr class="odd">
<td>Controle ótimo (iLQG, SQL e H∞)</td>
<td>(CROUSAZ; FARSHIDIAN; BUCHLI, 2014; CROUSAZ et al., 2015; RAFFO; ALMEIDA, 2016)</td>
</tr>
<tr class="even">
<td>Controle preditivo com base em modelo (<em>Model Predictive Control – MPC)</em></td>
<td>(ALEXIS et al., 2016; NOTTER et al., 2016; ZÜRN et al., 2016)</td>
</tr>
<tr class="odd">
<td>Controle adaptativo</td>
<td>(BISGAARD; LA COUR-HARBO; DIMON BENDTSEN, 2010; DAI; LEE; BERNSTEIN, 2014; FENG et al., 2015)</td>
</tr>
</tbody>
</table>

Vale ressaltar que, apesar da separação apresentada, muitas vezes as soluções não são aplicadas isoladamente. Muitos trabalhos apresentam soluções híbridas, agregando os dois tipos de técnica, como também será feito neste trabalho.

### Controle em malha aberta

As técnicas de controle de malha aberta atuam na modificação do sinal de referência com base em informações prévias do comportamento do sistema sem fazer uso de informações realimentadas por sensores. As principais técnicas aplicadas a este sistema são: *input shaping*, otimização de trajetória, aprendizado por reforço e método analítico com base na definição de sistema diferencialmente plano.

*Input shaping* se baseia na ideia de provocar oscilações transientes no sistema e cancelá-las logo em seguida através da inserção de uma entrada que produziria uma oscilação oposta. Isso é feito através da convolução do sinal de referência com sinais impulsivos adequadamente selecionados com base na frequência natural do sistema (QIAN; YI, 2015). Diversos trabalhos (BISGAARD; COUR-HARBO; BENDTSEN, 2008; KLAUSEN; FOSSEN; JOHANSEN, 2015, 2017), aplicam este filtro a sinais de referência arbitrários, reduzindo substancialmente as oscilações em comparação a solução apenas com controle em malha fechada.

Palunko; Fierro; Cruz (2012) aplicam procedimento de otimização *offline* por programação dinâmica para determinar trajetórias de referência que, com base em funções custo obtidas sobre a resposta de um modelo linearizado e discreto do sistema, visam minimizar oscilações da resposta. Faust et al. (2013, 2017) desenvolvem um gerador de trajetórias sem oscilação por meio de um algoritmo de aprendizado por reforço de iteração de valor aproximado, de modo que a política inferida se estende a domínios além da situação de treinamento, sendo robusto a ruídos e incertezas no modelo.

Sreenath; Lee; Kumar (2013) e Sreenath; Michael; Kumar (2013) demonstram que o sistema do *drone* com carga suspensa é diferencialmente plano. Isso significa que, conhecido um conjunto de variáveis de saída definida como funções diferenciáveis até determinada ordem, é possível determinar todas as outras variáveis do sistema e os esforços de entrada.

### Controle em Malha Fechada

Visto que o sistema é não linear, grande parte das soluções de controle são baseadas em ferramentas de análise de sistemas não lineares, como no critério de estabilidade de *Lyapunov*. Por exemplo, Klausen; Fossen; Johansen (2015, 2017) desenvolvem um controlador *backstepping* que garante cumprimento de trajetórias arbitrárias independente do movimento do pêndulo. Kui et al (2017) e Zhou et al. (2016) também buscam controlar a posição do drone compensando, além dos distúrbios causados pela carga, distúrbios externos com limiares conhecidos, aplicando controle por modos deslizantes. Já Guerrero et al. (2015a, 2015b, 2017a) aplicam controlador baseado no princípio de passividade de forma a minimizar a oscilação da carga sem medir a sua posição diretamente, uma vez que a lei de controle não depende dela.

Muitas técnicas utilizam ou se baseiam em técnicas de otimização. Por exemplo, Crousaz; Farshidian; Buchli (2014) e Crousaz et al. (2015) aplicam técnicas similares de controle linear quadrático ótimo: iLQG e SQL, que consideram a otimização da trajetória e do desempenho do controle em malha fechada de forma conjunta. Raffo; Almeida (2016) aplicam *H∞* para controlar a posição da carga sob condições de incerteza de parâmetros e distúrbios externos. Outros autores propõem diversas variações de controle preditivo baseado em modelo, que se baseiam no princípio de gerar entradas ótimas que consideram o estado presente e o comportamento previsto do sistema em um horizonte finito (ALEXIS et al., 2016; NOTTER et al., 2016; ZÜRN et al., 2016).

Muitos autores também aplicam controlador geométrico, que evita singularidades e definição de coordenadas no sistema, sendo popular no controle de *drones* (sem carga) para executar manobras agressivas (LEE; LEOK; MCCLAMROCH, 2010). Goodarzi; Lee; Lee (2014) consegue controlar o *drone* para realizar manobras ágeis ao mesmo tempo que estabiliza a posição do conjunto cabo e carga, modelado como um braço de múltiplas juntas em série. Sreenath; Lee; Kumar (2013); Sreenath; Michael; Kumar (2013) já propõem controle da posição da carga com trajetórias definidas com base na propriedade de planicidade diferencial do sistema.

## Objetivo e Contribuições

Observa-se que o sistema em análise atende a diversas demandas da sociedade e apresenta uma complexidade que desafia os pesquisadores da área de dinâmica e controle. Existe uma quantidade significativa de trabalhos recentes a respeito, porém observou-se que ainda não há uma solução dominante e que há muito espaço para explorar novas técnicas. Diante deste cenário, este trabalho visa desenvolver uma nova solução que traz inovações principalmente na definição do controlador de malha fechada e na estratégia de geração de trajetórias.

Basicamente, o presente trabalho aplica uma variação específica da técnica de controle por modos deslizantes para controlar a posição do *drone* compensando distúrbios externos e os ocasionados pela movimentação da carga, juntamente com a combinação de duas técnicas de malha aberta para mitigar a oscilação da carga: *input shaping* e geração de trajetória com base na propriedade de planicidade diferencial do sistema.

O controle por modos deslizantes (CMD) já foi aplicado com sucesso significativo no controle de *drones* sem carga e de pontes rolantes. Dois artigos recentes de revisão sobre controle de *drones* (MO; FARID, 2018; ÖZBEK; ÖNKOL;EFE, 2016) selecionam controladores com base em modos deslizantes como melhor custo-benefício dentre as soluções analisadas e diversos trabalhos de revisão de controle de pontes rolantes apontam esta técnica com uma das principais para a aplicação (QIAN; YI, 2015; RAMLI et al., 2017).

Apesar do sucesso nestas aplicações relacionadas, observou-se que esta técnica foi pouco explorada na aplicação alvo deste trabalho. Encontrou-se apenas dois trabalhos que utilizam controle por modos deslizantes diretamente (KUI et al., 2017; ZHOU et al., 2016). Ambos aplicam a técnica com o intuito de controlar a posição do *drone* compensando os distúrbios externos e os causados pela oscilação da carga, porém, não visam a estabilização desta.

Especificamente, a técnica de controle desenvolvida neste trabalho se baseia em (ZHENG; XIONG; LUO, 2014) e (XIONG; ZHENG, 2014), que realizam o controle de um *quadrotor* sem carga. Em comparação a estes trabalhos de referência, a solução desenvolvida inova ao adicionar o efeito da carga suspensa ao modelo, ao propor uma definição alternativa às variáveis deslizantes e ao desenvolver uma estratégia própria de determinação dos parâmetros de controle para que o sistema seja localmente estável com base no critério estabilidade de *Routh-Hurwitz*.

Em relação à estratégia de geração de trajetórias, o presente trabalho testa a aplicação de *input shaping* em trajetórias polinomiais definidas para o drone e para carga. No caso em que se define uma trajetória para a carga, obtém-se a referência para o drone fazendo-se uso da propriedade de planicidade diferencial do sistema. Demonstra-se que esta combinação é uma alternativa simples a estratégias elaboradas de otimização para gerar trajetórias para o drone que sejam factíveis de serem seguidas pelo controlador. Nenhum trabalho que combina estas duas técnicas para esta aplicação foi identificado.

## Estrutura do Documento

O presente trabalho se estrutura em seis capítulos:

- **Capítulo I:** presente seção, em que se apresentou as motivações por trás da pesquisa, uma breve revisão bibliográfica e a apresentação dos pontos de contribuição da pesquisa;

- **Capítulo II:** dedução do modelo dinâmico do sistema por meio dos métodos de *Newton-Euler* e *Euler-Lagrange*.

- **Capítulo III:** apresenta a solução de controle desenvolvida com detalhe no tratamento com a natureza de sub-atuação do sistema e na análise de estabilidade, apresentando também uma verificação do comportamento do sistema controlado em simulação;

- **Capítulo IV:** discorre sobre as técnicas de geração de trajetória com base na propriedade de planicidade diferencial do sistema e *input shaping* para então apresentar a solução combinada proposta;

- **Capítulo V:** análise do desempenho do controlador em simulação para diferentes configurações de trajetória, incluindo a nova combinação proposta,

- **Capítulo VI:** compilação dos resultados e indicação de trabalhos futuros.

2.  

# MODELO DINÂMICO

Este capítulo apresenta o desenvolvimento do modelo dinâmico do sistema que, em linhas gerais, é obtido por meio da aplicação as equações de *Newton-Euler* e *Lagrange*. Parte-se do entendimento da dinâmica de um *drone* sem carga para então derivar as equações do sistema completo de modo que os efeitos da adição da carga fiquem evidenciados.

## Drone sem Carga

A Figura 2.1 esquematiza um *quadcóptero*, indicando os sistemas de coordenadas e os esforços aplicados ao mesmo.

<img src="media/image4.emf" style="width:3.00473in;height:2.63296in" />

Figura 2.1 - Representação esquemática de drone sem carga com indicação dos sistemas de coordenadas, das forças e momentos aplicados, além do sentido de rotação das hélices.

Como indicado na Figura 2.1, define-se um referencial inercial $\Sigma_{i} = \left\lbrack {\overrightarrow{e}}_{x},{\overrightarrow{e}}_{y},{\overrightarrow{e}}_{z} \right\rbrack$ na origem do sistema e um não inercial (ou do corpo) $\Sigma_{b} = \left\lbrack {\overrightarrow{e}}_{x}^{b},{\overrightarrow{e}}_{y}^{b},{\overrightarrow{e}}_{z}^{b} \right\rbrack$ localizado no centro de massa da aeronave. A sua posição em relação ao referencial inercial é representada por $\overrightarrow{r} = \lbrack x,y,z\rbrack^{T}$, o vetor $\overrightarrow{v} = \lbrack u,v,w\rbrack^{T}$ representa a velocidade linear no referencial inercial e o vetor $\overrightarrow{\omega} = \lbrack p,q,r\rbrack^{T}$ representa sua velocidade angular no referencial não inercial. A massa do veículo é representada por $M$, enquanto $d$ é a distância entre um rotor e o seu oposto, e $g$ é a aceleração gravitacional.

A orientação do veículo é definida pelos ângulos de Euler $\overrightarrow{\eta} = \lbrack\phi,\theta,\psi\rbrack^{T}$, também chamados ângulos de rolagem, arfagem e guinada, de modo que o sistema de coordenadas não inercial é obtido por meio de três rotações consecutivas em torno dos eixos $z$, $y$ e $x$ respectivamente. A transformação de grandezas vetoriais definidas no referencial do corpo para o referencial inercial é dada pela matriz de transformação (APÊNDICE I – TRANSFORMAÇÕES CINEMÁTICAS):

|                                                                                                                         |       |
|-------------------------------------------------------------------------------------------------------------------------|-------|
| $$\mathbf{R} = \begin{bmatrix}                                                                                          
 \cos\theta\cos\psi & \sin\phi\sin\theta\cos\psi - \cos\phi\sin\psi & \sin\phi\sin\theta + \cos\phi\sin\theta\cos\psi \\  
 \cos\theta\sin\psi & \sin\phi\sin\theta\sin\psi + \cos\phi\cos\psi & \cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi \\    
  - \sin\theta & \sin\phi\cos\theta & \cos\phi\cos\theta                                                                  
 \end{bmatrix}$$                                                                                                          | (2.1) |

Já a transformação entre as velocidades angulares no referencial não inercial e a taxa de variação dos ângulos de Euler é dada pela matriz de transformação (APÊNDICE I – TRANSFORMAÇÕES CINEMÁTICAS):

|                                                               |       |
|---------------------------------------------------------------|-------|
| $$\left\lbrack \begin{array}{r}                               
 \dot{\phi} \\                                                  
 \dot{\theta} \\                                                
 \dot{\psi}                                                     
 \end{array} \right\rbrack = \begin{bmatrix}                    
 1 & \sin\phi{tg}\theta & \cos\phi{tg}\theta \\                 
 0 & \cos\phi & - \sin\phi \\                                   
 0 & \frac{\sin\phi}{\cos\theta} & \frac{\cos\phi}{\cos\theta}  
 \end{bmatrix}\left\lbrack \begin{array}{r}                     
 p \\                                                           
 q \\                                                           
 r                                                              
 \end{array} \right\rbrack$$                                    | (2.2) |

A Figura 2.1 também indica os principais esforços existentes na aeronave. As principais forças atuantes no sistema são a força peso $- Mg{\overrightarrow{e}}_{z}$ e as forças de propulsão dos rotores, indicadas como $F_{1},F_{2},F_{3}$ e $F_{4}$, aplicadas ao longo de ${\overrightarrow{e}}_{z}^{b}$. A força de propulsão resultante é dada por:

|                                                                             |       |
|-----------------------------------------------------------------------------|-------|
| $$F_{z}^{b} = F_{1} + F_{2} + F_{3} + F_{4}$$                               | (2.3) |
| $${\overrightarrow{F}}_{b} = \left\lbrack 0,0,F_{z}^{b} \right\rbrack^{T}$$ | (2.4) |

A aeronave também sofre ação de momento nos três eixos: as forças de propulsão provocam momento em torno dos eixos ${\overrightarrow{e}}_{x}^{b}$ e ${\overrightarrow{e}}_{y}^{b}$, enquanto o arrasto nas hélices, que atua contrário ao movimento de rotação das mesmas, provoca momento em torno do eixo ${\overrightarrow{e}}_{z}^{b}$. Sendo $d$ a distância entre um rotor e o seu oposto e $\tau_{i}$ o momento de arrasto em cada hélice, o momento resultante na aeronave a dado por:

|                                                                                                         |       |
|---------------------------------------------------------------------------------------------------------|-------|
| $$\tau_{x}^{b} = d\left( F_{2} - F_{4} \right)$$                                                        | (2.5) |
| $$\tau_{y}^{b} = d\left( F_{1} - F_{3} \right)$$                                                        | (2.6) |
| $$\tau_{z}^{b} = \tau_{1} - \tau_{2} + \tau_{3} - \tau_{4}$$                                            | (2.7) |
| $${\overrightarrow{\tau}}_{b} = \left\lbrack \tau_{x}^{b},\tau_{y}^{b},\tau_{z}^{b} \right\rbrack^{T}$$ | (2.8) |

As forças de propulsão e o torque nas hélices devido ao arrasto, por sua vez, são proporcionais ao quadrado da velocidade de rotação de cada hélice $\Omega_{i}$:

|                                    |        |
|------------------------------------|--------|
| $$F_{i} = k_{t}\Omega_{i}^{2}$$    | (2.9)  |
| $$\tau_{i} = k_{m}\Omega_{i}^{2}$$ | (2.10) |

As constantes $k_{t}$ e $k_{m}$ dependem da densidade do ar, do raio, formato, número e geometria das hélices, além dos coeficientes de arrasto e sustentação associados (PROUTY, 2001).

Dessa forma, o mapeamento da velocidade dos rotores (variável de comando aos controladores dos motores) e os esforços aplicados no drone, que serão as entradas do controlador a ser detalhado, é dado por:

|                                                 |        |
|-------------------------------------------------|--------|
| $$\overrightarrow{u} = \left\{ \begin{array}{r} 
 u_{1} \\                                         
 u_{2} \\                                         
 u_{3} \\                                         
 u_{4}                                            
 \end{array} \right\} = \left\{ \begin{array}{r}  
 F_{z}^{b} \\                                     
 \tau_{x}^{b} \\                                  
 \tau_{y}^{b} \\                                  
 \tau_{z}^{b}                                     
 \end{array} \right\} = \begin{bmatrix}           
 k_{t} & k_{t} & k_{t} & k_{t} \\                 
 0 & dk_{t} & 0 & - dk_{t} \\                     
 dk_{t} & 0 & - dk_{t} & 0 \\                     
 k_{m} & - k_{m} & k_{m} & - k_{m}                
 \end{bmatrix}\left\{ \begin{array}{r}            
 \Omega_{1}^{2} \\                                
 \Omega_{2}^{2} \\                                
 \Omega_{3}^{2} \\                                
 \Omega_{4}^{2}                                   
 \end{array} \right\}$$                           | (2.11) |

Ressalta-se que, visto este mapeamento direto entre a atuação dos rotores e a força e momento resultante na aeronave, toma-se o problema de controle até a definição destes esforços.

Com isso, o modelo dinâmico do *quadcóptero* é obtido aplicando-se as equações de *Newton-Euler*. A dinâmica translacional é escrita no referencial inercial e obtida igualando-se a taxa de variação do movimento linear à somatória das forças externas:

|                                                                                                                                                          |        |
|----------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\frac{d}{dt}\left( M\dot{\overrightarrow{r}} \right) = \sum_{}^{}{\overrightarrow{F}}_{ext}$$                                                          | (2.12) |
| $$M\ \ddot{\overrightarrow{r}} = \mathbf{R}{\overrightarrow{F}}_{b} - Mg{\overrightarrow{e}}_{z} + {\overrightarrow{F}}_{d} + {\overrightarrow{D}}_{F}$$ | (2.13) |

Na Eq. (2.13), ${\overrightarrow{F}}_{d}$ refere-se à força de arrasto translacional na aeronave, modelada como proporcional à velocidade do *drone* (FREDDI; LANZON; LONGHI, 2011):

|                                                                                                              |        |
|--------------------------------------------------------------------------------------------------------------|--------|
| $${\overrightarrow{F}}_{d} = {- \left\lbrack c_{x}\dot{x},{\ c}_{y}\dot{y},c_{z}\dot{z} \right\rbrack}^{T}$$ | (2.14) |

Sendo $c_{x}$, $c_{y}$ e $c_{z}$ os coeficientes de arrasto translacional em cada direção. ${\overrightarrow{D}}_{F}$ refere-se aos distúrbios de força não modelados. Isolando-se os termos de aceleração da Eq. (2.13), tem-se:

|                                                                                                                                |        |
|--------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\left\{ \begin{aligned}                                                                                                      
  & \ddot{x} = \frac{1}{M}\left( \cos\phi\sin\theta\cos\psi + \sin\phi\sin\psi \right)u_{1} - \frac{c_{x}}{M}\dot{x} + d_{x} \\  
  & \ddot{y} = \frac{1}{M}\left( \cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi \right)u_{1} - \frac{c_{y}}{M}\dot{y} + d_{y} \\  
  & \ddot{z} = - g + \frac{1}{M}\left( \cos\phi\cos\theta \right)u_{1} - \frac{c_{z}}{M}\dot{z} + d_{z}                          
 \end{aligned} \right.\ $$                                                                                                       | (2.15) |

Na Eq. (2.15), $d_{x}$, $d_{y}$ e $d_{z}$ refere-se aos efeitos do distúrbio ${\overrightarrow{D}}_{F}$ em cada componente da aceleração de translação.

Já a dinâmica rotativa é tomada no referencial do corpo e obtida igualando-se a taxa de variação do momento angular à somatória dos momentos externos:

|                                                                                                                                                                                           |        |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\frac{d}{dt}\left( \mathbf{I}\overrightarrow{\omega} \right) = \sum_{}^{}{\overrightarrow{\tau}}_{ext}^{b}$$                                                                            | (2.16) |
| $$\mathbf{I}\dot{\overrightarrow{\omega}} + \overrightarrow{\omega} \times \left( \mathbf{I}\overrightarrow{\omega} \right) = {\overrightarrow{\tau}}^{b} + {\overrightarrow{D}}_{\tau}$$ | (2.17) |

Na Eq. (2.17), $\mathbf{I}$ é a matriz de inércia do drone, tomada como $\text{diag}\text{ }\left( I_{x},I_{y},I_{z} \right)$. A taxa de variação do momento angular inclui, além do termo da aceleração angular ($\dot{\overrightarrow{\omega}}$) e uma parcela referente à variação da direção do momento angular do drone ($\overrightarrow{\omega} \times \mathbf{I}\overrightarrow{\omega}$). Já os momentos externos incluem, além de ${\overrightarrow{\tau}}^{b}$ proveniente da Eq. (2.8) e os distúrbios de momento não modelados (${\overrightarrow{D}}_{\tau}$).

Isolando os termos de aceleração da Eq. (2.17), tem-se:

|                                                                                                   |        |
|---------------------------------------------------------------------------------------------------|--------|
| $$\left\{ \begin{aligned}                                                                         
 \dot{p} & = \frac{\left( I_{y} - I_{z} \right)}{I_{x}}qr + \frac{1}{I_{x}}u_{2} + D_{\tau}^{x} \\  
 \dot{q} = \frac{\left( I_{z} - I_{x} \right)}{I_{y}}pr + \frac{1}{I_{y}}u_{3} + D_{\tau}^{y} \\    
 \dot{r} & = \frac{\left( I_{x} - I_{y} \right)}{I_{z}}pq + \frac{1}{I_{z}}u_{4} + D_{\tau}^{z}     
 \end{aligned} \right.\ $$                                                                          | (2.18) |

Na perspectiva de controle, porém, trabalha-se com a taxa de variação dos ângulos de Euler $\overrightarrow{\Omega}$, o que requer o uso da transformação descrita na Eq. (2.2). Esta transformação, por sua vez, gera alta complexidade às equações, valendo a tomada de uma simplificação comumente feita na literatura: $\lbrack p,q,r\rbrack \approx \lbrack\dot{\phi},\dot{\theta},\dot{\psi}\rbrack$, que é exata para o ponto de equilíbrio em que $\phi = 0$ e $\theta = 0$, resultando-se em:

|                                                                                                                         |        |
|-------------------------------------------------------------------------------------------------------------------------|--------|
| $$\left\{ \begin{aligned}                                                                                               
 \ddot{\phi} & = \frac{\left( I_{y} - I_{z} \right)}{I_{x}}\dot{\theta}\dot{\psi} + \frac{1}{I_{x}}u_{2} + d_{\phi} \\    
 \ddot{\theta} & = \frac{\left( I_{z} - I_{x} \right)}{I_{y}}\dot{\phi}\dot{\psi} + \frac{1}{I_{y}}u_{3} + d_{\theta} \\  
 \ddot{\psi} & = \frac{\left( I_{x} - I_{y} \right)}{I_{z}}\dot{\phi}\dot{\theta} + \frac{1}{I_{z}}u_{4} + d_{\psi}       
 \end{aligned} \right.\ $$                                                                                                | (2.19) |

A fim de simplificar a notação do distúrbio na Eq. (2.19), tornando-a mais conveniente para uso do controlador, resume-se os termos de distúrbio resultante nas acelerações como $d_{\phi}$, $d_{\theta}$ e $d_{\psi}$.

Portanto, o modelo dinâmico do *drone* sem carga pode ser resumido pela dinâmica de translação descrita no sistema de equações (2.15) e pela dinâmica rotativa, descrita no sistema de equações (2.19).

## *Drone* com Carga Suspensa por Cabo

O modelo dinâmico desenvolvido considera o *drone* como um corpo rígido com a uma carga pontual ligada ao seu centro de massa por meio de um cabo de massa desprezível e restrito a estar sempre tensionado. Dessa forma, os efeitos elásticos do cabo, a orientação da carga e a sua interferência na dinâmica rotativa do *drone* são desprezadas. A massa suspensa não gera perturbações de momento no *drone*, apenas de força. Ressalta-se que estas considerações vão de acordo com grande parte dos trabalhos relacionados encontrados na literatura (GUERRERO-SÁNCHEZ et al., 2017b; KLAUSEN; FOSSEN; JOHANSEN, 2017; SREENATH; MICHAEL; KUMAR, 2013).

A figura a seguir ilustra o sistema indicando os elementos adicionais ao sistema composto apenas pelo *drone*.

<img src="media/image5.emf" style="width:2.72077in;height:2.71021in" />

Figura 2.2 - Representação esquemática do drone com carga suspensa por cabo, indicando os referenciais, a posição da carga, além da força de tração do cabo e o peso da carga.

A posição da carga é representada por ${\overrightarrow{r}}_{L} = \left\lbrack x_{L},y_{L},z_{L} \right\rbrack^{T}$ e se relaciona com a posição do *drone* como:

|                                                                           |        |
|---------------------------------------------------------------------------|--------|
| $${\overrightarrow{r}}_{L} = \overrightarrow{r} + l\ \overrightarrow{p}$$ | (2.20) |

$l$ corresponde ao comprimento do cabo e $\overrightarrow{p}$ consiste no vetor unitário que aponta do centro de gravidade do quadcóptero para a carga, sendo obtido por meio de duas rotações consecutivas do vetor $- {\overrightarrow{e}}_{z}$: uma rotação de ângulo $\phi_{L}$ em torno de $x$ seguida de outra rotação de ângulo $\theta_{L}$ em torno do eixo $y$:

|                                                                                                           |        |
|-----------------------------------------------------------------------------------------------------------|--------|
| $$\overrightarrow{p} = R_{x}\left( \phi_{L} \right)R_{y}\left( \theta_{L} \right)\left\{ \begin{array}{r} 
 0 \\                                                                                                       
 0 \\                                                                                                       
  - 1                                                                                                       
 \end{array} \right\}$$                                                                                     | (2.21) |
| $$\overrightarrow{p} = \left\{ \begin{array}{r}                                                           
  - \sin\theta_{L} \\                                                                                       
 \sin\left( \phi_{L} \right)\cos\left( \theta_{L} \right) \\                                                
  - \cos\left( \phi_{L} \right)\cos\left( \theta_{L} \right)                                                
 \end{array} \right\}$$                                                                                     | (2.22) |

Ao longo deste vetor ocorre a aplicação da força de tração do cabo no drone ($\overrightarrow{T} = T\overrightarrow{p}$), assumida ser não-nula a todo instante. Também há a aplicação da força peso na carga e no *drone*, além da força de propulsão nas hélices e a força de arrasto.

Assim, aplicando-se as equações de Newton para o drone e para a carga, tem-se:

|                                                                                                                                                                             |        |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$M\ddot{\overrightarrow{r}} = \mathbf{R}{\overrightarrow{F}}_{b} - Mg{\overrightarrow{e}}_{z} + \overrightarrow{T} + {\overrightarrow{F}}_{d} + {\overrightarrow{D}}_{F}$$ | (2.23) |
| $$m{\ddot{\overrightarrow{r}}}_{L} = - \overrightarrow{T} - mg{\overrightarrow{e}}_{z} + {\overrightarrow{F}}_{d}^{L} + {\overrightarrow{D}}_{F}^{L}$$                      | (2.24) |

Substituindo ${\overrightarrow{r}}_{L}$ e suas derivadas da Eq. ((2.20)) na Eq. (2.24) e $\overrightarrow{T}$ da Eq. (2.24) na Eq. (2.23)[^1], obtém-se:

|                                                                                                                                                                                                                                                                |        |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$(M + m)\ddot{\overrightarrow{r}} + ml\ddot{\overrightarrow{p}} + (M + m)g{\overrightarrow{e}}_{z} = \mathbf{R}{\overrightarrow{F}}_{b} + {\overrightarrow{F}}_{d} + {\overrightarrow{F}}_{d}^{L} + {\overrightarrow{D}}_{F} + {\overrightarrow{D}}_{F}^{L}$$ | (2.25) |

$\left\lbrack {\overrightarrow{F}}_{d},{\overrightarrow{F}}_{d}^{L} \right\rbrack$ e $\left\lbrack {\overrightarrow{D}}_{F},{\overrightarrow{D}}_{F}^{L} \right\rbrack$ referem-se à força de arrasto e distúrbios de força aplicados ao *drone* e à carga respectivamente. A força de arrasto na carga também é modelada como proporcional à velocidade, como foi feito para ao drone (Eq. 2.14), porém simétrico nas três direções:

|                                                                          |        |
|--------------------------------------------------------------------------|--------|
| $${\overrightarrow{F}}_{d}^{L} = - c_{L}{\dot{\overrightarrow{r}}}_{L}$$ | (2.26) |

Visto as premissas consideradas para o modelo, a adição da carga tem efeito apenas na dinâmica de translação no *drone*, Eq. (2.15), de forma que a dinâmica rotativa, Eq. (2.19) permanece inalterada. Além disso, como a carga é considerada uma massa pontual, sua dinâmica rotativa é desconsiderada.

Desenvolvendo a Eq. (2.24), obtém-se um conjunto de três equações em função não somente das variáveis de estado do *drone* $\left\lbrack \dot{x},\dot{y},\dot{z},\phi,\theta,\psi,\dot{\phi},\dot{\theta},\dot{\psi} \right\rbrack$, como também do estado da carga, descrita em função de $\left\lbrack \phi_{L},\theta_{L},{\dot{\phi}}_{L},{\dot{\theta}}_{L},{\ddot{\phi}}_{L},{\ddot{\theta}}_{L} \right\rbrack$. Porém, na formulação de *Newton-Euler* o comportamento destas variáveis não fica evidente. Assim, aplica-se a formulação de *Lagrange* para detalhar o modelo obtido.

Para isso, define-se o *Lagrangiano* do sistema, dado pela diferença das energias cinética e potencial do sistema:

|                                                                                                                                                                                                     |        |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$L = \frac{M}{2}\left( {\dot{x}}^{2} + {\dot{y}}^{2} + {\dot{z}}^{2} \right) + \frac{m}{2}\left( {\dot{x}}_{L}^{2} + {\dot{y}}_{L}^{2} + {\dot{z}}_{L}^{2} \right) - g\left( Mz + mz_{L} \right)$$ | (2.27) |

Assim, o *Lagrangiano* é desenvolvido substituindo-se ${\overrightarrow{r}}_{L}$, Eq. (2.20) e sua derivada na Eq. (2.27). Com isso, as relações dinâmicas do sistema são obtidas aplicando-se a equação de Lagrange com base nas coordenadas generalizadas $\overrightarrow{q} = \left\lbrack x,y,z,\phi_{L},\theta_{L} \right\rbrack$, como mostra a Eq. (2.28), sendo $Τ_{i}$ os esforços generalizados ao longo de cada coordenada:

|                                                                                                                                    |        |
|------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\frac{\partial}{\partial t}\left( \frac{\partial L}{\partial{\dot{q}}_{i}} \right) - \frac{\partial L}{\partial q_{i}} = Τ_{i}$$ | (2.28) |

Analisando os esforços presentes nas Equações (2.23) e (2.24), as força de propulsão e arrasto no *drone* já estão descritas ao longo de $x$, $y$ e $z$. Porém, a força de arrasto na carga está descrita em função das suas coordenadas cartesianas e, portanto, devem ser convertidas para as coordenadas generalizadas. Para isso, define-se a função potencial associada às forças de arrasto como:

|                                                                                                                                                                                              |        |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$P = - \frac{1}{2}\left\lbrack c_{x}{\dot{x}}^{2} + c_{y}{\dot{y}}^{2} + c_{z}{\dot{z}}^{2} + c_{L}\left( {\dot{x}}_{L}^{2} + {\dot{y}}_{L}^{2} + {\dot{z}}_{L}^{2} \right) \right\rbrack$$ | (2.29) |

Assim, o esforço generalizado associado à força de arrasto translacional ao longo de cada coordenada $i$ é dado por:

|                                                          |        |
|----------------------------------------------------------|--------|
| $$Τ_{i}^{P} = \frac{\partial P}{\partial{\dot{q}}_{i}}$$ | (2.30) |

Por conveniência, os termos de distúrbio desconhecidos são transferidos diretamente para cada coordenada. Desse modo, as equações da dinâmica do sistema resultante são dadas por:

|                                                                                                                                                                                                                                         |        |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\left\{ \begin{aligned}                                                                                                                                                                                                               
  & \frac{\partial}{\partial t}\left( \frac{\partial L}{\partial\dot{x}} \right) - \frac{\partial L}{\partial x} & = & \left( \cos\phi\sin\theta\cos\psi + \sin\phi\sin\psi \right)u_{1} + \frac{\partial P}{\partial\dot{x}} + D_{x} \\  
  & \frac{\partial}{\partial t}\left( \frac{\partial L}{\partial\dot{y}} \right) - \frac{\partial L}{\partial y} & = & \left( \cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi \right)u_{1} + \frac{\partial P}{\partial\dot{y}} + D_{y} \\  
  & \frac{\partial}{\partial t}\left( \frac{\partial L}{\partial\dot{z}} \right) - \frac{\partial L}{\partial z} & = & \left( \cos\phi\cos\theta \right)u_{1} + \frac{\partial P}{\partial\dot{z}} + D_{z} \\                             
  & \frac{\partial}{\partial t}\left( \frac{\partial L}{\partial{\dot{\phi}}_{L}} \right) - \frac{\partial L}{\partial\phi_{L}} & = & \frac{\partial P}{\partial{\dot{\phi}}_{L}} + D_{\phi_{L}} \\                                       
  & \frac{\partial}{\partial t}\left( \frac{\partial L}{\partial{\dot{\theta}}_{L}} \right) - \frac{\partial L}{\partial\theta_{L}} & = & \frac{\partial P}{\partial{\dot{\theta}}_{L}} + D_{\theta_{L}}                                  
 \end{aligned} \right.\ $$                                                                                                                                                                                                                | (2.31) |

Desenvolvendo-se o sistema de equações (2.31), o sistema pode ser escrito na forma matricial:

|                                                                                                                                                                                                                                                                                                                                                             |        |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\mathbf{M}\left( \overrightarrow{q} \right)\ddot{\overrightarrow{q}} + \mathbf{C}\left( \overrightarrow{q},\dot{\overrightarrow{q}} \right)\dot{\overrightarrow{q}} + \mathbf{G}\left( \overrightarrow{q} \right) = \mathbf{B}\left( \overrightarrow{q} \right)u_{1} + \mathbf{P}\left( \overrightarrow{q} \right)\dot{\overrightarrow{q}} + \mathbf{D}$$ | (2.32) |

Em que:

|                                                                                                                                                                                                                 |        |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\mathbf{M}\left( \overrightarrow{q} \right) = \begin{bmatrix}                                                                                                                                                 
 (M + m) & 0 & 0 & 0 & - ml\ c\theta_{L} \\                                                                                                                                                                       
 0 & (M + m) & 0 & ml\ c\theta_{L}\ c\theta_{L} & - ml\ s\phi_{L}\ c\theta_{L} \\                                                                                                                                 
 0 & 0 & (M + m) & ml\ s\phi_{L}\ c\theta_{L} & ml\ c\phi_{L}\ s\theta_{L} \\                                                                                                                                     
 0 & ml\ c\theta_{L}\ c\theta_{L} & ml\ s\phi_{L}\ c\theta_{L} & ml^{2}\ {c\theta_{L}}^{2} & 0 \\                                                                                                                 
  - ml\ c\theta_{L} & - ml\ s\phi_{L}\ c\theta_{L} & ml\ c\phi_{L}\ s\theta_{L} & 0 & ml^{2}                                                                                                                      
 \end{bmatrix}$$                                                                                                                                                                                                  | (2.33) |
| $$\mathbf{C}\left( \overrightarrow{q},\dot{\overrightarrow{q}} \right) = \left\lbrack \mathbf{Ο}_{\mathbf{5 \times 3}} \middle| \begin{matrix}                                                                  
 0 & ml\ s\theta_{L}\ {\dot{\theta}}_{L} \\                                                                                                                                                                       
  - ml\left( s\phi_{L}c\theta_{L}\ {\dot{\phi}}_{L} + c\phi_{L}s\theta_{L}\ {\dot{\theta}}_{L} \right) & - ml\left( s\phi_{L}c\theta_{L}\ {\dot{\theta}}_{L} + c\phi_{L}s\theta_{L}\ {\dot{\phi}}_{L} \right) \\  
 ml\left( c\phi_{L}c\theta_{L}\ {\dot{\phi}}_{L} - s\phi_{L}s\theta_{L}\ {\dot{\theta}}_{L} \right) & ml\left( c\phi_{L}c\theta_{L}\ {\dot{\theta}}_{L} - s\phi_{L}s\theta_{L}\ {\dot{\phi}}_{L} \right) \\       
  - ml^{2}s\theta_{L}c\theta_{L}{\dot{\theta}}_{L} & - ml^{2}s\theta_{L}c\theta_{L}{\dot{\phi}}_{L} \\                                                                                                            
 ml^{2}s\theta_{L}c\theta_{L}{\dot{\phi}}_{L} & 0                                                                                                                                                                 
 \end{matrix} \right\rbrack$$                                                                                                                                                                                     | (2.34) |
| $$\mathbf{G}\left( \overrightarrow{q} \right) = \left\lbrack \begin{array}{r}                                                                                                                                   
 0 \\                                                                                                                                                                                                             
 0 \\                                                                                                                                                                                                             
 (M + m)g \\                                                                                                                                                                                                      
 mgl\sin\phi_{L}\cos\theta_{L} \\                                                                                                                                                                                 
 mgl\cos\phi_{L}\sin\theta_{L}                                                                                                                                                                                    
 \end{array} \right\rbrack$$                                                                                                                                                                                      | (2.35) |
| $$\mathbf{B}\left( \overrightarrow{q} \right) = \left\lbrack \begin{array}{r}                                                                                                                                   
 u_{x} \\                                                                                                                                                                                                         
 u_{y} \\                                                                                                                                                                                                         
 u_{z} \\                                                                                                                                                                                                         
 0 \\                                                                                                                                                                                                             
 0                                                                                                                                                                                                                
 \end{array} \right\rbrack = \left\lbrack \begin{array}{r}                                                                                                                                                        
 \cos\phi\sin\theta\cos\psi + \sin\phi\sin\psi \\                                                                                                                                                                 
 \cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi \\                                                                                                                                                                 
 \cos\phi\cos\theta \\                                                                                                                                                                                            
 0 \\                                                                                                                                                                                                             
 0                                                                                                                                                                                                                
 \end{array} \right\rbrack$$                                                                                                                                                                                      | (2.36) |
| $$\mathbf{P}\left( \overrightarrow{q} \right) = \begin{bmatrix}                                                                                                                                                 
  - \left( C_{x} + C_{L} \right) & 0 & 0 & 0 & C_{L}lc\theta_{L} \\                                                                                                                                               
 0 & - \left( C_{y} + C_{L} \right) & 0 & - C_{L}lc\phi_{L}c\theta_{L} & C_{L}ls\phi_{L}s\theta_{L} \\                                                                                                            
 0 & 0 & - \left( C_{x} + C_{L} \right) & - C_{L}ls\phi_{L}c\theta_{L} & - C_{L}lc\phi_{L}s\theta_{L} \\                                                                                                          
 0 & - C_{L}lc\phi_{L}c\theta_{L} & - C_{L}ls\phi_{L}c\theta_{L} & - C_{L}l^{2}c\theta_{L}^{2} & 0 \\                                                                                                             
 C_{L}lc\theta_{L} & C_{L}ls\phi_{L}s\theta_{L} & - C_{L}lc\phi_{L}s\theta_{L} & 0 & - C_{L}l^{2}                                                                                                                 
 \end{bmatrix}$$                                                                                                                                                                                                  | (2.37) |

Observa-se que $\mathbf{M}\left( \overrightarrow{q} \right)$ é uma matriz positiva definida, ou seja, é simétrica e os termos da diagonal principal são estritamente positivos exceto para quando $\theta_{L} = \frac{\pi}{2}$. Com isso, toma-se como restrição que $\theta_{L}$ e $\phi_{L}$ sejam menores que $\frac{\pi}{2}$, de forma que o cabo se movimente sempre abaixo da do nível da aeronave. Dado este cenário, é possível isolar o termo de aceleração $\ddot{\overrightarrow{q}}$ da Eq. (2.32):

|                                                                                                                                                                                                                                                                                                                                                                                            |        |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\ddot{\overrightarrow{q}} = \mathbf{M}^{- 1}\left( \overrightarrow{q} \right)\left\lbrack - \mathbf{C}\left( \overrightarrow{q},\dot{\overrightarrow{q}} \right)\dot{\overrightarrow{q}} - \mathbf{G}\left( \overrightarrow{q} \right) + \mathbf{B}\left( \overrightarrow{q} \right)u + \mathbf{P}\left( \overrightarrow{q} \right)\dot{\overrightarrow{q}} + \mathbf{D} \right\rbrack$$ | (2.38) |

Para fins de controle, opta-se por englobar o termo referente ao arrasto como distúrbio, de modo que as equações resultantes do desenvolvimento da Eq. (2.38) com esta consideração são dadas por:

|                                                                                                                                                                                                          |        |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\left\{ \begin{aligned}                                                                                                                                                                                
  & \ddot{x} & = & f_{x}\left( \theta_{L},{\dot{\phi}}_{L},{\dot{\theta}}_{L} \right) & + & b_{x}\left( \phi,\theta,\psi,\phi_{L},\theta_{L} \right)\  & u_{1} & + d_{x} \\                                
  & \ddot{y} & = & f_{y}\left( \phi_{L},\theta_{L},{\dot{\phi}}_{L},{\dot{\theta}}_{L} \right) & + & b_{y}\left( \phi,\theta,\psi,\phi_{L},\theta_{L} \right) & u_{1} & + d_{y} \\                         
  & \ddot{z} & = & f_{z}\left( \phi_{L},\theta_{L},{\dot{\phi}}_{L},{\dot{\theta}}_{L} \right) & + & b_{z}\left( \phi,\theta,\psi,\phi_{L},\theta_{L} \right)\  & u_{1} & + d_{z} \\                       
  & {\ddot{\phi}}_{L} & = & f_{\phi_{L}}\left( \theta_{L},{\dot{\phi}}_{L},{\dot{\theta}}_{L} \right) & + & b_{\phi_{L}}\left( \phi,\theta,\psi,\phi_{L},\theta_{L} \right)\  & u_{1} & + d_{\phi_{L}} \\  
  & {\ddot{\theta}}_{L} & = & f_{\theta_{L}}\left( \theta_{L},{\dot{\phi}}_{L} \right) & + & b_{\theta_{L}}\left( \phi,\theta,\psi,\phi_{L},\theta_{L} \right)\  & u_{1} & + d_{\theta_{L}}                
 \end{aligned} \right.\ $$                                                                                                                                                                                 | (2.39) |

O último termo de cada equação do sistema (2.39) se refere ao efeito dos distúrbios (incluindo o arrasto) sobre as acelerações.

Enfim, vale ressaltar que a expansão dos termos de aceleração da Eq. (2.39) é útil para a implementação do controlador, visto que ele faz uso integral do modelo escrito nesta forma. Assim, segue a equação expandida de cada um dos termos da Eq. (2.39):

|                                                                                                                                                                                                                     |        |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$f_{x} = - \frac{ml\sin\theta_{L}}{(M + m)}\left( \cos^{2}\theta_{L}{\dot{\phi}}_{L}^{2} + {\dot{\theta}}_{L}^{2} \right)$$                                                                                        | (2.40) |
| $$b_{x} = \frac{m}{M(M + m)}\left\lbrack s\theta_{L}c\theta_{L}\left( u_{y}s\phi_{L} - u_{z}c\phi_{L}\  \right) + u_{x}\left( \frac{M}{m} + {c\theta_{L}}^{2} \right) \right\rbrack$$                               | (2.41) |
| $$f_{y} = \frac{ml\sin\phi_{L}\cos\theta_{L}}{(M + m)}\left( \cos^{2}\theta_{L}{\dot{\phi}}_{L}^{2} + {\dot{\theta}}_{L}^{2} \right)$$                                                                              | (2.42) |
| $$b_{y} = \frac{m}{M(M + m)}\left\lbrack s\phi_{L}c\theta_{L}\left( u_{x}s\theta_{L} + u_{z}c\phi_{L}c\theta_{L} \right) + u_{y}\left( \frac{M}{m} + 1 - {s\phi_{L}}^{2}{c\theta_{L}}^{2} \right) \right\rbrack$$   | (2.43) |
| $$f_{z} = - \frac{ml\cos\phi_{L}\cos\theta_{L}}{(M + m)}\left( \cos^{2}\theta_{L}{\dot{\phi}}_{L}^{2} + {\dot{\theta}}_{L}^{2} \right) - g$$                                                                        | (2.44) |
| $$b_{z} = \frac{m}{M(M + m)}\left\lbrack c\phi_{L}c\theta_{L}\left( - u_{x}s\theta_{L} + u_{y}s\phi_{L}c\theta_{L} \right) + u_{z}\left( \frac{M}{m} + 1 - {c\phi_{L}}^{2}{c\theta_{L}}^{2} \right) \right\rbrack$$ | (2.45) |
| $$f_{\phi_{L}} = 2{tg}\theta_{L}{\dot{\phi}}_{L}{\dot{\theta}}_{L}$$                                                                                                                                                | (2.46) |
| $$b_{\phi_{L}} = - \frac{\left( u_{y}\cos\phi_{L} + u_{z}\sin\phi_{L} \right)}{Ml\cos\theta_{L}}$$                                                                                                                  | (2.47) |

3.  

# CONTROLE

Este capítulo apresenta a solução de controle desenvolvida para o sistema *drone* com carga suspensa por cabo descrito no Capítulo 2. Como introduzido anteriormente, deseja-se controlar a posição da aeronave e ao mesmo tempo manter a posição da carga estável. Para isso, desenvolve-se um controlador por modos deslizantes que controla a posição do *drone* que considera a dinâmica acoplada da carga, mas não visa a estabilização dela. A tarefa de estabilização da carga é deixada para o gerador de trajetórias descrito no próximo capítulo.

Primeiramente, apresenta-se uma análise qualitativa do modelo dinâmico e do problema de controle, avaliando as características de atuação do sistema e fazendo referência a outras soluções de controle pertinentes. Depois, parte-se para o detalhamento do controlador, apresentando desde a formulação e a análise de estabilidade até a verificação do comportamento do sistema em simulação.

## Introdução

### Características de Atuação

Analisando as equações da dinâmica do *drone* com carga, verifica-se que o sistema possui seis graus de liberdade ($\overrightarrow{q} = \left\lbrack x,y,z,\phi,\theta,\psi,\phi_{L},\theta_{L} \right\rbrack^{T}$) para quatro entradas de controle independentes ($\overrightarrow{u} = \left\lbrack u_{1},u_{2},u_{3},u_{4} \right\rbrack^{T}$). Esta diferença caracteriza o sistema como sub-atuado, o que significa que a ação de controle não é capaz de atuar em todos os graus de liberdade de forma independente. Comparado ao sistema composto apenas pelo *drone*, o desafio proposto adiciona dois graus de liberdades não atuados ao problema.

Verifica-se que $u_{2}$, $u_{3}$ e $u_{4}$, os esforços de torque no *drone*, aparecem explicitamente nas equações das variáveis que descrevem a sua orientação $\phi$, $\theta$ e $\psi$ (Eq. (2.19)). Isso significa que, isoladamente, é possível controlar as três variáveis por meio destas três entradas. Porém, $\phi$ e $\theta$ também configuram a orientação da força de propulsão, que é responsável por provocar o deslocamento da aeronave. Portanto, os sinais de entrada $u_{2}$ e $u_{3}$ afetam indiretamente posição $\lbrack x,y,z\rbrack$. Esta influência também pode ser verificada constatando a presença dos ângulos nas equações da dinâmica de translação da aeronave (Eq. (2.15)).

A força de propulsão $u_{1}$, por sua vez, está explicitamente presente nas equações das variáveis que descrevem a posição do drone $\lbrack x,y,z\rbrack$ e da carga $\left\lbrack \phi_{L},\theta_{L} \right\rbrack$ (Eq. (2.39)). Porém, $u_{1}$ aponta verticalmente no ponto de equilíbrio, em torno do qual se deseja manter as variáveis do sistema, exercendo influência somente na aceleração $\ddot{z}$ nesta condição. Ou seja, nas condições de operação mais frequentes, a propulsão exerce controle majoritariamente ao longo de $z$.

Estas características de atuação motivaram pesquisadores da área de controle de aeronaves multi-rotoras a desenvolver soluções de controle em cascata como ilustrado na figura seguir.

<img src="media/image6.emf" style="width:5.502in;height:1.99852in" />

Figura 3.1 – Estrutura de controle em cascata para multicópteros. Adaptado de MO; FARID (2018)

Basicamente, a solução apresenta um controlador de posição em cascata a um controlador de atitude (ou orientação). Dadas as posições desejadas, o controlador de posição gera o sinal $u_{1}$ e valores de referência para os ângulos de rolagem e arfagem ($\phi_{d}$ e $\theta_{d}$) que, juntamente com a orientação desejada $\psi_{d}$, alimentam o controlador de atitude que gera os sinais $u_{2}$, $u_{3}$ e $u_{4}$. Geralmente, $u_{4}$ é determinado isoladamente com base no ângulo de guinada desejado. O deslocamento nas direções $x$ e $y$ é alcançado por meio da ação de $u_{2}$ e $u_{3}$, que direcionam a força de propulsão $u_{1}$ para a direção de redução do erro de deslocamento, como ilustra a Figura 3.2.

<img src="media/image7.emf" style="width:3.64493in;height:2.13275in" />

Figura 3.2 - Ilustração do efeito de $u_{2}$ e $u_{3}$ sobre o deslocamento horizontal do drone.

Como ilustrado na Figura 3.2, a ação de $u_{2}$ direciona a propulsão no sentido de deslocar a aeronave ao longo de ${\overrightarrow{e}}_{y}^{b}$, enquanto a ação de $u_{3}$ tem influência indireta sobre o deslocamento ao longo de ${\overrightarrow{e}}_{x}^{b}$.

### Controle por Modos Deslizantes (CMD)

#### Conceitos Básicos

O CMD é uma técnica de controle não linear robusta, ou seja, insensível a distúrbios externos e incertezas de parâmetros, cuja implementação se resume a:

1\) Definir as chamadas variáveis deslizantes, que são funções das variáveis do sistema cuidadosamente projetadas para que, quando se anulem, o sistema apresente comportamento estável;

2\) Projetar os sinais de entrada de modo a conduzir as variáveis deslizantes a zero e mantê-las nesta condição.

Por exemplo, dado um sistema não linear escrito na forma:

|                      |        |
|----------------------|--------|
| $$\dot{x} = f(x,u)$$ | (3.48) |

sendo $x$ a variável do sistema e $u$ o sinal de entrada, pode-se definir, por exemplo, uma variável deslizante como uma combinação linear da variável de estado e a sua derivada:

|                                             |        |
|---------------------------------------------|--------|
| $$s = \dot{x} + \lambda x,\ \ \lambda > 0$$ | (3.49) |

Observa-se que, quando $s = 0$, tem-se:

|                                              |        |
|----------------------------------------------|--------|
| $${\dot{x} = - \lambda x                     
 }{x = x(0)e^{- \lambda t}                     
 }{\dot{x} = - \lambda x(0)e^{- \lambda t}}$$  | (3.50) |

Nesta situação, $x$ e $\dot{x}$ convergem para zero assintoticamente. Assim, quando o sinal de controle é definido de forma adequada, o retrato de fase do sistema para quando se define as variáveis deslizantes como feito no exemplo da Eq. (3.49) se assemelha ao mostrado na Figura 3.3.

<img src="media/image8.emf" style="width:3.58953in;height:2.42462in" />

Figura 3.3 - Retrato de fase característico de um sistema comandado por um controlador por modos deslizantes para variável deslizante linear. Adaptado de GHAZALI et al. (2011).

Como mostra a Figura 3.3, o estado em que $s = 0$ corresponde à reta indicada no retrato de fase. Este estado é denominado superfície ou modo deslizante. Considerando $s \neq 0$ no estado inicial, o sistema controlado primeiramente é conduzido até a superfície deslizante, executando a chamada “fase de aproximação”, e então segue deslizando ao longo da superfície até o ponto de equilíbrio, executando a chamada “fase de deslizamento”.

Existem diversas soluções em torno deste conceito. De modo geral, as versões de CMD diferenciam-se pela forma com que determinam as variáveis deslizantes e pela estratégia que usam para realizar as fases de aproximação e deslizamento. Para conhecer mais sobre a técnica e as suas variações, recomenda-se as fontes (QIAN; YI, 2015; SHTESSEL et al., 2013; UTKIN; GULDNER; SHI, 2009).

#### CMD Aplicado a Drones com Carga Suspensa por Cabo

Foram encontrados dois trabalhos na literatura que aplicam controle por modos deslizantes a este sistema (KUI et al., 2017; ZHOU et al., 2016). Basicamente, eles utilizam o mesmo princípio de atuação comumente utilizado para drones como mostra a Figura 3.1 (Seção 3.1.1).

(KUI et al., 2017) assumem a existência de uma força aplicada à aeronave com componentes independentes ao longo de cada eixo do sistema de coordenadas inercial. A partir de análise geométrica deste vetor através da Eq. (2.36), é possível determinar qual deve ser a força de propulsão e os ângulos de rolagem e arfagem ideais para produzir esta entrada. Dessa forma, é possível tratar o sistema como totalmente atuado, de modo que cada componente da força virtual é projetada para controlar por modos deslizantes a posição da aeronave ao longo de cada eixo. A força de propulsão resultante da transformação geométrica é passada adiante, enquanto os ângulos de rolagem e arfagem calculadas são passados como referência para um controlador de atitude que também aplica controle por modos deslizantes clássico para cada eixo, resultando nos comandos de torque a serem enviados ao sistema de atuação da aeronave.

Observa-se que, nesta abordagem, a determinação dos ângulos de arfagem e rolagem fica totalmente em função da saída do controlador de posição. Esta estrutura da possibilidade de utilizar valores de referência determinados externamente, como é feito neste trabalho com o gerador de trajetórias.

Também vale ressaltar que os artigos não deixam claro a interdependência entre as acelerações do drone e da carga na equação dinâmica. Considerando o modelo dinâmico em função das coordenadas generalizadas, Eq. (2.39), as “forças virtuais” mencionadas não se manifestam de forma exata de se traduzir em um estado de orientação desejado como acontece nas equações do drone sem carga.

Em contrapartida, o controlador desenvolvido neste trabalho leva em conta a característica de sub-atuação do sistema de forma explícita, sem usar o recurso da força virtual; permite a determinação externa de referências para os ângulos de rolagem e arfagem; e consideram o modelo dinâmico completo, considerando todas as interações mútuas entre *drone* e carga.

## Estratégia de Controle

Como descrito no Capítulo 2, as acelerações do sistema são descritas pelos sistemas de equações (2.39) e (2.19), que representam as dinâmicas de translação e rotação respectivamente. Para o intuito de realizar o controle, o sistema é subdividido em dois, um totalmente atuado, formado pelas variáveis $z$ e $\psi$, e outro sub-atuado, formado pelas variáveis $x$, $y$, $\phi$ e $\theta$:

|                                                                                   |        |
|-----------------------------------------------------------------------------------|--------|
| $$\left\{ \begin{aligned}                                                         
 \ddot{z} & = f_{x} + b_{x}u_{1} + d_{z} \\                                         
 \ddot{\psi} & = f_{\psi} + b_{\psi}u_{4} + d_{\psi}                                
 \end{aligned} \right.\ \ \ \ \ \ \ \ \ \ \ \text{(Subistema totalmente atuado)}$$  | (3.51) |
| $$\left\{ \begin{aligned}                                                         
 \ddot{x} & = f_{x} + b_{x}u_{1} + d_{x} \\                                         
 \ddot{y} & = f_{y} + b_{y}u_{1} + d_{y} \\                                         
  \\                                                                                
 \ddot{\phi} & = f_{\phi} + b_{\phi}u_{2} + d_{\psi} \\                             
 \ddot{\theta} & = f_{\theta} + b_{\theta}u_{3} + d_{\theta}                        
 \end{aligned} \right.\ \ \ \ \ \ \ \ \ \ \ (\text{Subsistema sub-atuado)}$$        | (3.52) |

O estado da carga, que está diretamente associado às variáveis $\phi_{L}$ e $\theta_{L}$, não é controlado diretamente. Sua estabilização é atingida por meio da geração de trajetórias adequadas, como será descrito na Seção 4.

Neste contexto, a solução desenvolvida é composta por dois grupos de controladores por modos deslizantes em cascata, como ilustra a Figura 3.4.

<img src="media/image9.emf" style="width:4.46481in;height:2.66038in" />

Figura 3.4 - Estrutura de controle do sistema drone com carga suspensa por cabo.

Como ilustrado na Figura 3.4, o primeiro CMD se encarrega de controlar o subsistema totalmente atuado com base nos valores desejados para altitude e ângulo de guinada até suas segundas derivadas, determinando $u_{1}$ e $u_{4}$. O segundo controlador comanda o subsistema sub-atuado com base nas referências de posição horizontal e ângulos de rolagem e arfagem até a segunda derivada, além de $u_{1}$ obtido anteriormente, gerando os sinais de controle restantes: $u_{2}$ e $u_{3}$.

### CMD do Subsistema Totalmente Atuado ($z,\psi$)

Para realizar o controle de altitude e guinada da aeronave, aplica-se a variação clássica de controlador por modos deslizantes aplicado a sistemas não lineares, reproduzindo o que já foi feito por outros autores (XIONG; ZHENG, 2014; ZHENG; XIONG; LUO, 2014).

#### Dedução do Controlador

Primeiramente, define-se as variáveis deslizantes:

|                                                                                                         |        |
|---------------------------------------------------------------------------------------------------------|--------|
| $$s_{1} = \left( {\dot{z}}_{d} - \dot{z} \right) + \lambda_{z}\left( z_{d} - z \right)$$                | (3.53) |
| $$s_{2} = \left( {\dot{\psi}}_{d} - \dot{\psi} \right) + \lambda_{\psi}\left( \psi_{d} - \psi \right)$$ | (3.54) |

O objetivo do CMD é conduzir estas variáveis até zero para que, uma vez nesta condição, as variáveis do subsistema se estabilizem de modo que $z \rightarrow z_{d}$ e $\psi \rightarrow \psi_{d}$. De fato, quando $s_{1} = 0$, tem-se:

|                                                                                                          |        |
|----------------------------------------------------------------------------------------------------------|--------|
| $${\left( {\dot{z}}_{d} - \dot{z} \right) = - \lambda_{z}\left( z_{d} - z \right)                        
 }{\left( z_{d} - z \right) = \left( z_{d} - z \right)(0)e^{- \lambda_{z}t}                                
 }{\left( {\dot{z}}_{d} - \dot{z} \right) = - \lambda_{z}\left( z_{d} - z \right)(0)e^{- \lambda_{z}t}}$$  | (3.55) |

Dessa forma, $\left( z_{d} - z \right) \rightarrow 0$ e $\left( {\dot{z}}_{d} - \dot{z} \right) \rightarrow 0$ assintoticamente. O mesmo acontece para $s_{2} = 0$, em que $\left( \psi_{d} - \psi \right) \rightarrow 0$ e $\left( {\dot{\psi}}_{d} - \dot{\psi} \right) \rightarrow 0$ assintoticamente.

O próximo passo consiste em definir as entradas do sistema que realizem a regularização das variáveis deslizantes. Para isso, primeiramente, extrai-se as derivadas de $s_{1}$ e $s_{2}$:

|                                                                                                                                 |        |
|---------------------------------------------------------------------------------------------------------------------------------|--------|
| $${\dot{s}}_{1} = \left( {\ddot{z}}_{d} - \ddot{z} \right) + \lambda_{z}\left( z_{d} - z \right)$$                              | (3.56) |
| $${\dot{s}}_{2} = \left( {\ddot{\psi}}_{d} - \ddot{\psi} \right) + \lambda_{\psi}\left( {\dot{\psi}}_{d} - \dot{\psi} \right)$$ | (3.57) |

Observa-se que ${\dot{s}}_{1}$ e ${\dot{s}}_{2}$ contêm as acelerações $\ddot{z}$ e $\ddot{\psi}$, estas definidas em função das entradas $u_{1}$ e $u_{4}$ segundo o sistema de equações (3.51). Assim, é possível conduzir ${\dot{s}}_{1}$ e ${\dot{s}}_{2}$ de forma a estabilizar $s_{1}$ e $s_{2}$ como desejado. Com isso, deseja-se que:

|                                                                                                            |        |
|------------------------------------------------------------------------------------------------------------|--------|
| $${\dot{s}}_{1} = - \kappa_{1}s_{1} - \eta_{1}{sign}{\left( s_{1} \right),\ \ \kappa_{1},\eta_{1} > 0\ }$$ | (3.58) |
| $${\dot{s}}_{2} = - \kappa_{2}s_{2} - \eta_{2}{sign}\left( s_{2} \right),\ \ \kappa_{2},\eta_{2} > 0$$     | (3.59) |

Onde:

|                                       |        |
|---------------------------------------|--------|
| $${sign}(x) = \left\{ \begin{aligned} 
 1,\ \ \  & se\ x \geq 0 \\             
  - 1,\ \ \  & se\ x < 0                
 \end{aligned} \right.\ $$              | (3.60) |

Substituindo (3.56) e (3.57) em (3.58) e (3.59) dadas as acelerações escritas em função das entradas conforme a Eq. (3.51) sem os distúrbios, encontra-se:

|                                                                                                                    |        |
|--------------------------------------------------------------------------------------------------------------------|--------|
| $$u_{1} = \frac{{\ddot{z}}_{d} - f_{z} + \kappa_{1}s_{1} + \eta_{1}{sign}\left( s_{1} \right)}{b_{z}}$$            | (3.61) |
| $$u_{4} = \frac{{\ddot{\psi}}_{d} - f_{\psi} + \kappa_{2}s_{2} + \eta_{2}{sign}\left( s_{2} \right)\ }{b_{\psi}}$$ | (3.62) |

#### Análise de Estabilidade

A motivação por trás da a definição imposta para ${\dot{s}}_{1}$ e ${\dot{s}}_{2}$ se dá com base na teoria de estabilidade de *Lyapunov*, que estabelece o critério de que, dado um sistema escrito na forma $\dot{x} = f(x)$ com $x = 0$ como ponto de equilíbrio, uma função $V(x):\mathbb{R}^{n}\mathbb{\rightarrow R}$ é chamada função de *Lyapunov* candidata e o sistema é estável no sentido de *Lyapunov* se:

1)  $V(x) = 0$ se e somente se $x = 0$;

2)  $V(x) > 0$ para todo $x \neq 0$;

3)  $\dot{V}(x) \leq 0$ $\rightarrow$ o sistema é localmente estável;

4)  $\dot{V}(x) < 0$ para todo $x \neq 0 \rightarrow$ o sistema é assintoticamente estável;

Uma função de *Lyapunov* candidata pode ser interpretada como uma função de energia do sistema que é sempre dissipada ao longo do tempo até a nulidade, quando o sistema atinge o ponto de equilíbrio (QIAN; YI, 2015). Com isso, define-se as seguintes funções de *Lyapunov* candidatas:

|                                                      |        |
|------------------------------------------------------|--------|
| $$V_{1}\left( s_{1} \right) = \frac{1}{2}s_{1}^{2}$$ | (3.63) |
| $$V_{2}\left( s_{2} \right) = \frac{1}{2}s_{2}^{2}$$ | (3.64) |

Nota-se que as condições (a) $V(0) = 0$ e (b) $V(x) > 0\ (x \neq 0$) são satisfeitas, visto que $V_{1}$ e $V_{2}$ são funções quadráticas. Derivando-se $V_{1}$, tem-se:

|                                                                                                                                                                                                                                                                                     |        |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $${{\dot{V}}_{1} = s_{1}{\dot{\mathbf{s}}}_{\mathbf{1}}                                                                                                                                                                                                                             
 }{{\dot{V}}_{1} = s_{1}\left\lbrack \left( {\ddot{z}}_{d} - \ddot{\mathbf{z}} \right) + \lambda_{z}\left( {\dot{z}}_{d} - \dot{z} \right) \right\rbrack                                                                                                                              
 }{{\dot{V}}_{1} = s_{1}\left\lbrack \left( {\ddot{z}}_{d} - f_{z} - b_{z}\mathbf{u}_{\mathbf{1}} - d_{z} \right) + \lambda_{z}\left( {\dot{z}}_{d} - \dot{z} \right) \right\rbrack = s_{1}\left\lbrack - \kappa_{1}s_{1} - \eta_{1}{sign}\left( s_{1} \right) - d_{z} \right\rbrack  
 }{{\dot{V}}_{1} = - \kappa_{1}s_{1}^{2} - \eta_{1}\left| s_{1} \right| - \mathbf{d}_{\mathbf{z}}\mathbf{s}_{\mathbf{1}} \leq - \kappa_{1}s_{1}^{2} - \eta_{1}\left| s_{1} \right| + \left| d_{z} \right|\left| s_{1} \right|}$$                                                      | (3.65) |

Tomando-se $D_{z} = \max\left( \left| d_{z} \right| \right)$:

|                                                                                                                                                                                                                                                 |        |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $${{\dot{V}}_{1} \leq - \kappa_{1}s_{1}^{2} - \eta_{1}\left| s_{1} \right| + \left| \mathbf{d}_{\mathbf{z}} \right|\left| \mathbf{s}_{\mathbf{1}} \right| \leq - \kappa_{1}s_{1}^{2} - \eta_{1}\left| s_{1} \right| + D_{z}\left| s_{1} \right| 
 }{{\dot{V}}_{1} \leq - \kappa_{1}s_{1}^{2} + \left( D_{z} - \eta_{1} \right)\left| s_{1} \right|}$$                                                                                                                                              | (3.66) |

Observa-se que ${\dot{V}}_{1} < 0$ para $\kappa_{1} \geq 0$ e $\eta_{1} > D_{z}$. Similarmente, ${\dot{V}}_{2} < 0$ para $\kappa_{2} \geq 0$ e $\eta_{2} \geq D_{\psi}$, sendo $D_{\psi} = \max\left( \left| d_{\psi} \right| \right)$. Portanto, sob estas condições, o subsistema é assintoticamente estável no sentido de *Lyapunov* e robusto contra distúrbios que não ultrapassam os limites definidos.

#### Observações Relevantes

Um ponto a ser ressaltado é o de que o sinal de controle resultante é descontínuo, visto a presença da função ${sign}(s)$. Esta característica provoca uma entrada ruidosa que leva ao fenômeno chamado *chattering*. A Figura 3.5 apresenta o retrato de fase característico de um sistema controlado por modos deslizantes apresentando este efeito.

‘<img src="media/image10.emf" style="width:2.95283in;height:2.09375in" />

Figura 3.5 -Ilustração do efeito de *chattering* durante a fase de deslizamento. Adaptado de HOSSAIN et al. (2017).

Como mostrado na Figura 3.5, o termo descontínuo na entrada faz com que o sistema faça pequenos saltos em torno da superfície deslizante, caracterizando o fenômeno de *chattering*. Isso ocorre durante a fase de deslizamento, pois é quando a variável deslizante oscila em torno de zero, fazendo com que o sinal de controle chaveie entre $+ \eta$ e $- \eta$ devido ao termo ${sign}(s)$. Com isso, o sistema pode responder com oscilações indesejadas, podendo até leva-lo à instabilidade.

Esta característica é um dos pontos mais desvantajosos do controlador por modos deslizantes, porém existem formas de atenuá-lo. A forma mais simples é aproximar a função ${sign}(s)$ para uma função contínua aproximada, como a *sigmoide* e a tangente hiperbólico. Fazendo isso, porém, o controlador deixa de ser idealmente robusto, embora se consiga produzir bons resultados na prática (SHTESSEL et al., 2013).

Outra observação relevante é a de que o termo associado às constantes $\kappa_{1}$ e $\kappa_{2}$ não são estritamente necessários para garantir a convergência do sistema. A Figura 3.6 apresenta um exemplo de comportamento de uma variável deslizante, ilustrando a contribuição de cada termo.

<img src="media/image11.emf" style="width:3.80181in;height:2.85075in" />

Figura 3.6 - Exemplo de convergência de uma variável deslizante $s$ com decaimento exponencial, constante e combinado.

A Figura 3.6 mostra que $s$ apresenta decaimento linear para $\dot{s} = - \eta|s|$ e exponencial para $\dot{s} = - \kappa s$. A primeira parcela é responsável por garantir convergência em tempo finito e superar distúrbios externos (como demonstrado anteriormente), enquanto o principal papel da segunda é acelerar a convergência quando distante de zero.

### CMD do Subsistema Sub-atuado

Existem diversas variações de controle por modos deslizantes para controlar sistemas sub-atuado na literatura (ASHRAFIUON; ERWIN, 2004, 2008; SANKARANARAYANAN; MAHINDRAKAR, 2009; WANG et al., 2004; WANG; LIU; YI, 2007; XU; ÖZGÜNER, 2008). Basicamente, a estratégia para estender o uso da técnica para esse tipo de sistema consiste em definir variáveis deslizantes que combinam componentes sob influência direta dos sinais de controle com variáveis sem atuação direta e então determinar a entrada de modo a garantir a convergência destas superfícies e das variáveis do sistema durante a fase de deslizamento.

Especificamente, a técnica utilizada neste trabalho se baseia em (ZHENG; XIONG; LUO, 2014) e (XIONG; ZHENG, 2014), que realizam o controle de um quadcóptero sem carga. Ressalta-se que, em comparação a estes trabalhos, a solução desenvolvida, além de adicionar o efeito da carga suspensa ao modelo, inova ao propor definições alternativas para as variáveis deslizantes e os parâmetros de controle.

#### Dedução do Controlador

A intuição por trás do controlador é semelhante ao realizado no controle de quadcópteros, como ilustrado anteriormente na Figura 3.2: $u_{2}$ é direcionado a reduzir o erro ao longo de ${\overrightarrow{e}}_{y}^{b}$, enquanto $u_{3}$ atua no sentido de reduzir o erro ao longo de ${\overrightarrow{e}}_{x}^{b}$. Assim, calcula-se o erro de posição da aeronave projetado sobre no plano $xy$ como:

|                                        |        |
|----------------------------------------|--------|
| $$\left\{ \begin{array}{r}             
 {\widetilde{x}}_{b} \\                  
 {\widetilde{y}}_{b}                     
 \end{array} \right\} = \begin{bmatrix}  
 \cos\psi & \sin\psi \\                  
  - \sin\psi & \cos\psi                  
 \end{bmatrix}\left\{ \begin{array}{r}   
 x_{d} - x \\                            
 y_{d} - y                               
 \end{array} \right\}$$                  | (3.67) |

Na Eq. (3.67), assume-se $\psi$ como invariante no tempo, ou seja, $\dot{\psi} = \ddot{\psi} = 0$. Assim, as derivadas da Eq. (3.67) são dadas por:

|                                        |        |
|----------------------------------------|--------|
| $$\left\{ \begin{array}{r}             
 {\dot{\widetilde{x}}}_{b} \\            
 {\dot{\widetilde{y}}}_{b}               
 \end{array} \right\} = \begin{bmatrix}  
 \cos\psi & \sin\psi \\                  
  - \sin\psi & \cos\psi                  
 \end{bmatrix}\left\{ \begin{array}{r}   
 {\dot{x}}_{d} - \dot{x} \\              
 {\dot{y}}_{d} - \dot{y}                 
 \end{array} \right\}$$                  | (3.68) |
| $$\left\{ \begin{array}{r}             
 {\ddot{\widetilde{x}}}_{b} \\           
 {\ddot{\widetilde{y}}}_{b}              
 \end{array} \right\} = \begin{bmatrix}  
 \cos\psi & \sin\psi \\                  
  - \sin\psi & \cos\psi                  
 \end{bmatrix}\left\{ \begin{array}{r}   
 {\ddot{x}}_{d} - \ddot{x} \\            
 {\ddot{y}}_{d} - \ddot{y}               
 \end{array} \right\}$$                  | (3.69) |

Esta consideração se demonstra razoável, visto que as condições de controle de $\psi$ são favoráveis para atingir convergência em curto prazo (XIONG; ZHENG, 2014).

Com isso, define-se as variáveis deslizantes e suas derivadas como:

|                                                                                                                                                                                                                                |        |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$s_{3} = \lambda_{1}{\dot{\widetilde{x}}}_{b} + \lambda_{2}{\widetilde{x}}_{b} + \lambda_{3}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) + \lambda_{4}\left( \theta_{d} - \theta \right)$$                                | (3.70) |
| $$s_{4} = \lambda_{5}{\dot{\widetilde{y}}}_{b} + \lambda_{6}{\widetilde{y}}_{b} + \lambda_{7}\left( {\dot{\phi}}_{d} - \dot{\phi} \right) + \lambda_{8}\left( \phi_{d} - \phi \right)$$                                        | (3.71) |
| $${\dot{s}}_{3} = \lambda_{1}{\ddot{\widetilde{x}}}_{b} + \lambda_{2}{\dot{\widetilde{x}}}_{b} + \lambda_{3}\left( {\ddot{\theta}}_{d} - \ddot{\theta} \right) + \lambda_{4}\left( {\dot{\theta}}_{d} - \dot{\theta} \right)$$ | (3.72) |
| $${\dot{s}}_{4} = \lambda_{5}{\ddot{\widetilde{y}}}_{b} + \lambda_{6}{\dot{\widetilde{y}}}_{b} + \lambda_{7}\left( {\ddot{\phi}}_{d} - \ddot{\phi} \right) + \lambda_{8}\left( {\dot{\phi}}_{d} - \dot{\phi} \right)$$         | (3.73) |

Observa-se que as variáveis deslizantes são definidas como a combinação linear dos erros das variáveis do sistema. Diferentemente das variáveis deslizantes definidas para o subsistema totalmente atuado, a condição de estabilidade para quando $s_{3} = 0$ e $s_{4} = 0$ não é trivial. Neste momento, assume-se que a estabilidade será garantida e na Seção 3.2.2.3 é feita uma análise de estabilidade para a condição de deslizamento.

Observa-se que as equações que descrevem as derivadas das variáveis deslizantes contêm as acelerações $\ddot{\phi}$ e $\ddot{\theta}$, que são definidas em função das entradas $u_{2}$ e $u_{3}$ segundo o sistema de equações (3.52). Assim, espera-se ser possível conduzir ${\dot{s}}_{4}$ e ${\dot{s}}_{5}$ de forma a estabilizar $s_{4}$ e $s_{5}$ como desejado. Nota-se também os termos ${\ddot{\widetilde{x}}}_{b}$ e ${\ddot{\widetilde{y}}}_{b}$ possuem os termos $\ddot{x}$ e $\ddot{y}$, que são definidos em função de $u_{1}$, ao qual se atribui o valor já determinado pelo controlador de altitude para esta entrada, comportando-se como se fosse uma constante neste contexto (XIONG; ZHENG, 2014).

Assim, similarmente ao que foi feito para o subsistema totalmente atuado, deseja-se que:

|                                                                            |        |
|----------------------------------------------------------------------------|--------|
| $${\dot{s}}_{3} = - \kappa_{3}s_{3} - \eta_{3}{sign}\left( s_{3} \right)$$ | (3.74) |
| $${\dot{s}}_{4} = - \kappa_{4}s_{4} - \eta_{4}{sign}\left( s_{4} \right)$$ | (3.75) |

Igualando as equações (3.72) e (3.73) a (3.74) e (3.75), substituindo $\ddot{\psi}$ e $\ddot{\theta}$ dados pelo modelo (3.52) e isolando $u_{2}$ e $u_{3}$, obtém-se:

|                                                                                                                                                                                                                                                                                                          |        |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$u_{3} = \frac{\lambda_{1}{\ddot{\widetilde{x}}}_{b} + \lambda_{2}{\dot{\widetilde{x}}}_{b} + \lambda_{3}\left( {\ddot{\theta}}_{d} - f_{\theta} \right) + \lambda_{4}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) + \kappa_{3}s_{3} + \eta_{3}{sign}\left( s_{3} \right)}{\lambda_{3}b_{\theta}}$$ | (3.76) |
| $$u_{2} = \frac{\lambda_{5}{\ddot{\widetilde{y}}}_{b} + \lambda_{6}{\dot{\widetilde{y}}}_{b} + \lambda_{7}\left( {\ddot{\phi}}_{d} - f_{\phi} \right) + \lambda_{8}\left( {\dot{\phi}}_{d} - \dot{\phi} \right) + \kappa_{4}s_{4} + \eta_{4}{sign}\left( s_{4} \right)}{\lambda_{7}b_{\phi}}$$           | (3.77) |

#### Análise de Estabilidade das Variáveis Deslizantes

Como feito no CMD do subsistema totalmente atuado, a estabilidade das variáveis deslizantes é determinada pela teoria de estabilidade de *Lyapunov*. Assim, define-se as seguintes funções de *Lyapunov* candidatas:

|                                                      |        |
|------------------------------------------------------|--------|
| $$V_{3}\left( s_{3} \right) = \frac{1}{2}s_{3}^{2}$$ | (3.78) |
| $$V_{4}\left( s_{4} \right) = \frac{1}{2}s_{4}^{2}$$ | (3.79) |

Nota-se que as condições (a) $V(0) = 0$ e (b) $V(x) > 0\ (x \neq 0$) são satisfeitas, visto que $V_{3}$ e $V_{4}$ consistem em funções quadráticas. Derivando-se $V_{3}$, obtém-se:

|                                                                                                                                                                                                                                                                                                                                                                                                                                                     |        |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $${{\dot{V}}_{3} = s_{3}{\dot{\mathbf{s}}}_{\mathbf{3}} = s_{3}\left\lbrack \lambda_{1}{\ddot{\widetilde{\mathbf{x}}}}_{\mathbf{b}} + \lambda_{2}{\dot{\widetilde{x}}}_{b} + \lambda_{3}\left( {\ddot{\theta}}_{d} - \ddot{\mathbf{\theta}} \right) + \lambda_{4}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) \right\rbrack                                                                                                                     
 }{= s_{1}\left\lbrack \lambda_{1}\left( \cos\psi\left( {\ddot{x}}_{d} - f_{x} - b_{x}u_{1}\  - d_{x} \right) + \sin\psi\left( {\ddot{y}}_{d} - f_{y} - b_{y}u_{1}\  - d_{y} \right) \right)\  + \lambda_{2}{\dot{\widetilde{x}}}_{b}\ \ \  + \lambda_{3}\left( {\ddot{\theta}}_{d} - f_{\theta} - b_{\theta}\mathbf{u}_{\mathbf{3}} - d_{\theta} \right) + \lambda_{4}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) \right\rbrack                 
 }{{\dot{V}}_{3} = - \kappa_{3}s_{3}^{2} - \eta_{3}\left| s_{3} \right| - \left\lbrack \left( \lambda_{1} + \lambda_{3} \right)\left( \cos\psi d_{x} + \sin\psi d_{y} \right) + d_{\theta} \right\rbrack\mathbf{s}_{\mathbf{3}} \leq - \kappa_{3}s_{3}^{2} - \eta_{3}\left| s_{3} \right| + \left( \left| \lambda_{1} + \lambda_{3} \right|\left| \cos\psi d_{x} + \sin\psi d_{y} \right| + \left| d_{\theta} \right| \right)\left| s_{3} \right|}$$  | (3.80) |

Tomando-se $D_{{\widetilde{x}}_{b}} = \max\left( \left| \lambda_{1} + \lambda_{3} \right|\left| \cos\psi d_{x} + \sin\psi d_{y} \right| + \left| d_{\theta} \right| \right)$:

|                                                                                                                          |        |
|--------------------------------------------------------------------------------------------------------------------------|--------|
| $${{\dot{V}}_{3} \leq - \kappa_{3}s_{3}^{2} - \eta_{3}\left| s_{3} \right| + D_{{\widetilde{x}}_{b}}\left| s_{3} \right| 
 }{{\dot{V}}_{3} \leq - \kappa_{3}s_{3}^{2} + \left( D_{{\widetilde{x}}_{b}} - \eta_{3} \right)\left| s_{3} \right|}$$     | (3.81) |

Observa-se que ${\dot{V}}_{3} < 0$ para $\kappa_{3} \geq 0$ e $\eta_{3} > D_{{\widetilde{x}}_{b}}$. Similarmente, ${\dot{V}}_{4} < 0$ para $\kappa_{4} \geq 0$ e $\eta_{4} > D_{{\widetilde{y}}_{b}}$, sendo $D_{{\widetilde{y}}_{b}} = \max\left( \left| \lambda_{5} + \lambda_{7} \right|\left| - \sin\psi d_{x} + \cos\psi d_{y} \right| + \left| d_{\phi} \right| \right)$. Portanto, sob estas condições, o subsistema é assintoticamente estável no sentido de Lyapunov (para a variáveis $s_{3}$ e $s_{4}$) e robusto contra distúrbios que não ultrapassem os limites definidos.

#### Análise de Estabilidade do Sistema nas Superfícies Deslizantes

A forma mais trivial de provar a estabilidade do sistema durante a fase de deslizamento seria constatando que o sistema toma a forma $\dot{x} = - \mathbf{K}x$, onde $\mathbf{K}$ possui apenas valores positivos na diagonal principal, de forma que o sistema seja assintoticamente estável na variável $x$. Porém, a característica de subatuação e a forma de definição das variáveis deslizantes não favorece este cenário. Outra forma adequada seria definir uma função de Lyapunov $V(x)$ candidata na condição de deslizamento e constatar que $V(x) < 0$ (SANKARANARAYANAN; MAHINDRAKAR, 2009). Porém, esta abordagem não se demonstra trivial. Por fim, recorre-se à análise de estabilidade local em torno do ponto equilíbrio com base nos trabalhos de (ASHRAFIUON; ERWIN, 2008; ZHENG; XIONG; LUO, 2014).

A ideia central consiste em determinar os coeficientes $\lambda_{1}$ a $\lambda_{8}$ a partir da condição de estabilidade de *Routh-Hurwitz* aplicado às equações das superfícies deslizantes linearizadas em torno do ponto de equilíbrio.

##### Estabilidade no Deslizamento em $s_{3}$

Primeiramente, rearranja-se as equações (3.72) e (3.70) para as condições de deslizamento, em que ${\dot{s}}_{3} = 0$ e $s_{3} = 0$:

|                                                                                                                                                                                                                                                    |        |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $${\ddot{\theta}}_{d} - \ddot{\theta} = - \frac{\lambda_{1}}{\lambda_{3}}{\ddot{\widetilde{x}}}_{b} - \frac{\lambda_{2}}{\lambda_{3}}{\dot{\widetilde{x}}}_{b} - \frac{\lambda_{4}}{\lambda_{3}}\left( {\dot{\theta}}_{d} - \dot{\theta} \right)$$ | (3.82) |
| $${\dot{\widetilde{x}}}_{b} = - \frac{\lambda_{2}}{\lambda_{1}}{\widetilde{x}}_{b} - \frac{\lambda_{3}}{\lambda_{1}}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) - \frac{\lambda_{4}}{\lambda_{1}}\left( \theta_{d} - \theta \right)$$         | (3.83) |

Substituindo a Eq. (3.83) na Eq. (3.82), tem-se:

|                                                                                                                                                                                                                                                                                                                                                                                                        |        |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $${\ddot{\theta}}_{d} - \ddot{\theta} = - \frac{\lambda_{1}}{\lambda_{3}}{\ddot{\widetilde{x}}}_{b} + \frac{\lambda_{2}^{2}}{\lambda_{1}\lambda_{3}}{\widetilde{x}}_{b} + \left( \frac{\lambda_{2}}{\lambda_{1}} - \frac{\lambda_{4}}{\lambda_{3}} \right)\left( {\dot{\theta}}_{d} - \dot{\theta} \right) + \frac{\lambda_{2}\lambda_{4}}{\lambda_{1}\lambda_{3}}\left( \theta_{d} - \theta \right)$$ | (3.84) |

Faz-se então uma redefinição de variável, $y_{1} = \theta_{d} - \theta$, $y_{2} = {\dot{\theta}}_{d} - \dot{\theta}$ e $y_{3} = {\widetilde{x}}_{b}$, obtendo-se o sistema:

|                                                                                                                                                                                                                                                                                                                       |        |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\left\{ \begin{aligned}                                                                                                                                                                                                                                                                                             
 {\dot{y}}_{1} & = y_{2} \\                                                                                                                                                                                                                                                                                             
 {\dot{y}}_{2} & = - \frac{\lambda_{1}}{\lambda_{3}}{\ddot{\widetilde{x}}}_{b}\left( y_{1},y_{2} \right) + \frac{\lambda_{2}\lambda_{4}}{\lambda_{1}\lambda_{3}}y_{1} + \left( \frac{\lambda_{2}}{\lambda_{1}} - \frac{\lambda_{4}}{\lambda_{3}} \right)y_{2} + \frac{\lambda_{2}^{2}}{\lambda_{1}\lambda_{3}}y_{3} \\  
 {\dot{y}}_{3} & = - \frac{\lambda_{2}}{\lambda_{1}}y_{1} - \frac{\lambda_{3}}{\lambda_{1}}y_{2} - \frac{\lambda_{4}}{\lambda_{1}}y_{3}                                                                                                                                                                                 
 \end{aligned} \right.\ $$                                                                                                                                                                                                                                                                                              | (3.85) |

Na Eq. (3.85), ${\ddot{\widetilde{x}}}_{b}\left( y_{1},y_{2} \right)$ é dado pela Eq. (3.69) de modo que $\theta = \theta_{d} - y_{1}$ e $\dot{\theta} = {\dot{\theta}}_{d} - y_{2}$. Observa-se que, $y_{1} \rightarrow 0$, $y_{2} \rightarrow 0$ e $y_{3} \rightarrow 0$ quando as variáveis estão próximas dos seus pontos de equilíbrio, isto é, $\theta \rightarrow \theta_{d}$, $\dot{\theta} \rightarrow {\dot{\theta}}_{d}$ e ${\widetilde{x}}_{b} \rightarrow 0$. Definindo o vetor $\overrightarrow{y} = \left\{ y_{1},y_{2},y_{3} \right\}^{T}$, o ponto de equilíbrio ${\overrightarrow{y}}_{e} = \left\{ 0,0,0 \right\}$ e  $\dot{\overrightarrow{y}} = f\left( \overrightarrow{y} \right)$ (sistema 3.85), a linearização de $f\left( \overrightarrow{y} \right)$ em torno do ponto de equilíbrio é dada por:

|                                                                                                                                                                                       |        |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$f'\left( \overrightarrow{y} \right) = \left. \ \mathbf{J}(f) \right|_{\overrightarrow{y} = \left\{ 0,0,0 \right\}}\ \overrightarrow{y} + f\left( {\overrightarrow{y}}_{e} \right)$$ | (3.86) |

Na Eq. (3.86), $\left. \ \mathbf{J}(f) \right|_{\overrightarrow{y} = {\overrightarrow{y}}_{e}}$, ou simplesmente $\mathbf{J}$**,** é o jacobiano da função $f(y)$ avaliada no ponto de equilíbrio $y_{e}$, definido como:

|                                                                                                                                                                                                                                    |        |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\mathbf{J} = \begin{bmatrix}                                                                                                                                                                                                     
 \frac{\partial f_{1}}{\partial y_{1}} & \frac{\partial f_{1}}{\partial y_{2}} & \frac{\partial f_{1}}{\partial y_{3}} \\                                                                                                            
 \frac{\partial f_{2}}{\partial y_{1}} & \frac{\partial f_{2}}{\partial y_{2}} & \frac{\partial f_{2}}{\partial y_{3}} \\                                                                                                            
 \frac{\partial f_{3}}{\partial y_{1}} & \frac{\partial f_{3}}{\partial y_{2}} & \frac{\partial f_{3}}{\partial y_{3}}                                                                                                               
 \end{bmatrix}_{y = y_{e}} = \begin{bmatrix}                                                                                                                                                                                         
 0 & 1 & 0 \\                                                                                                                                                                                                                        
  - \frac{\lambda_{1}}{\lambda_{3}}F + \frac{\lambda_{2}\lambda_{4}}{\lambda_{1}\lambda_{3}} & \left( \frac{\lambda_{2}}{\lambda_{1}} - \frac{\lambda_{4}}{\lambda_{3}} \right) & \frac{\lambda_{2}^{2}}{\lambda_{1}\lambda_{3}} \\  
  - \frac{\lambda_{4}}{\lambda_{1}} & - \frac{\lambda_{3}}{\lambda_{1}} & - \frac{\lambda_{2}}{\lambda_{1}}                                                                                                                          
 \end{bmatrix}$$                                                                                                                                                                                                                     | (3.87) |

Dado que:

|                                                                                                                                                                                                                                          |        |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$F = \frac{\partial{\ddot{\widetilde{x}}}_{b}}{\partial y_{1}} = \frac{u_{1}c(\phi)m}{M(M + m)}\left\lbrack c\theta_{d}\ A\left( \psi,\phi_{L},\theta_{L} \right) - s\theta_{d}B\left( \psi,\phi_{L},\theta_{L} \right) \right\rbrack$$ | (3.88) |
| $$A\left( \psi,\phi_{L},\theta_{L} \right) = \frac{M}{m} + \left\lbrack \left( c\psi c\theta_{L} + s\psi s\phi_{L}s\theta_{L} \right)^{2} + s\psi^{2}c\phi_{L}^{2} \right\rbrack$$                                                       | (3.89) |
| $$B\left( \psi,\phi_{L},\theta_{L} \right) = c\phi_{L}c\theta_{L}\left( s\psi s\phi_{L}c\theta_{L} - c\psi s\theta_{L} \right)$$                                                                                                         | (3.90) |

Assim, o sistema linear descrito pela Eq. (3.86) é estável se os autovalores da matriz do jacobiano dado pela Eq. (3.87) forem todos menores que zero, de forma que $\overrightarrow{y}$ e  $\dot{\overrightarrow{y}}$ apresentem convergência assintótica até o ponto de equilíbrio. Espera-se que esta condição possa ser alcançada ajustando-se os coeficientes $\lambda_{1}$, $\lambda_{2}$, $\lambda_{3}$ e $\lambda_{4}$. Com isso, calcula-se o polinômio característico de $\mathbf{J}$, fazendo:

|                                                                                                                                                                          |        |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\det\left( p\mathbb{I} - \mathbf{J} \right) = 0$$                                                                                                                      | (3.91) |
| $$p^{3} + \left( \frac{\lambda_{4}}{\lambda_{3}} \right)p^{2} + \left( \frac{\lambda_{1}}{\lambda_{3}}F \right)p + \left( \frac{\lambda_{2}}{\lambda_{3}}F \right) = 0$$ | (3.92) |

Os autovalores são dados pelas raízes do polinômio característico. Porém, ao invés de calcular as raízes diretamente, aplica-se o critério de estabilidade de *Routh-Hurwitz*, que infere a estabilidade do sistema apenas avaliando os coeficientes do polinômio característico. Tendo o polinômio característico de terceiro grau escrito na forma $p^{3} + a_{2}p^{2} + a_{1}p + a_{0} = 0$, é condição necessária e suficiente para que o sistema linear invariante no tempo associado seja estável: $a_{2} > 0$, $a_{0} > 0$ e $a_{2}a_{1} > a_{0}$ (NISE, 2011). Portanto, para o polinômio característico (3.92):

|                                                                                              |        |
|----------------------------------------------------------------------------------------------|--------|
| $$1.\ \ \frac{\lambda_{4}}{\lambda_{3}} > 0$$                                                | (3.93) |
| $$2.\ \ \frac{\lambda_{2}}{\lambda_{3}}F > 0$$                                               | (3.94) |
| $$3.\ \ \frac{\lambda_{1}\lambda_{4}}{\lambda_{3}^{2}}F > \frac{\lambda_{2}}{\lambda_{3}}F$$ | (3.95) |

Observa-se que as duas últimas condições têm dependência do comportamento de $F$. Portanto, vale o estudo desta função para definir os parâmetros de controle. Assumindo que $u_{1} > 0$ (propulsão sempre positiva) e $- \frac{\pi\ }{2} < \left\lbrack \psi,\theta\text{,}\phi_{L},\theta_{L} \right\rbrack < \frac{\pi\ }{2}$, é possível fazer com que $F$ seja maior do que zero condicionando o valor dado a $\theta_{d}$, que é de domínio do gerador de trajetórias.

Primeiramente, nota-se que o termo em evidência da Eq. (3.88) ($u_{1}\cos\phi$) é sempre positivo para as condições de operação impostas. Em segundo, verifica-se que $\theta_{d}$ dá pesos para $A$ e $B$ de forma que $A$ é maximizado e $B$ neutralizado para $\theta_{d} = 0$ e o oposto ocorre para $\theta_{d} = \pm \frac{\pi}{2}$.

Outro comportamento observado é o de que $A\left( \psi,\phi_{L},\theta_{L} \right)$ é sempre positivo e o seu valor mínimo possível é $\frac{M}{m}$. Portanto, $\cos\theta_{d}A$ é maior do que $0$ nas condições de operação. Na contramão, o termo $B\left( \psi,\phi_{L},\theta_{L} \right)$ pode assumir valores positivos e negativos, porém limitados a $\pm \frac{1}{2}$ e, portanto, $\left| \sin\theta_{d}B \right|$ é menor do que $\frac{1}{2}$ nas condições de operação.

Este comportamento levou a suspeitar de que pudesse existir um valor máximo para $\theta_{d}$ que $F$ se tornaria positivo independentemente dos valores de $A$ e $B$. De fato, o caso em que $A$ assume seu valor mínimo e $B$ assume seu valor máximo (em módulo) retrata o cenário em que $F$ assume o menor valor possível. Assim, o valor de $\theta_{d}$ que anula $F$ nestas condições extremas ($\theta_{d}^{*}$) é obtido fazendo-se $F = 0$:

|                                                                                                                                                                                                                                       |        |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\cos{\left( \theta_{d}^{*} \right)\min\left( |A| \right)} - \sin\left( \theta_{d}^{*} \right)\max\left( |B| \right) = 0 \rightarrow \tan\left( \theta_{d}^{*} \right) = \pm \frac{\min\left( |A| \right)}{\max\left( |B| \right)}$$ | (3.96) |
| $$\theta_{d}^{*} = \pm {atan}\left( \frac{2M}{m} \right)$$                                                                                                                                                                            | (3.97) |

Portanto, pode-se afirmar que:

|                                                                                  |        |
|----------------------------------------------------------------------------------|--------|
| $$F > 0\ \ se\ \ \left| \theta_{d} \right| < {atan}\left( \frac{2M}{m} \right)$$ | (3.98) |

Na realidade, a definição deste limite é conservadora, visto que não há um estado $\left\lbrack \psi,\phi_{L},\theta_{L} \right\rbrack$ possível que faça com que $A$ e $B$ assumam seus valores de mínimo e máximo (em módulo) ao mesmo tempo. O limite real ocorre, dentre outros estados, quando $\psi = 0$, $\phi_{L} = 0$ e $\theta_{L} = \pm \frac{\pi}{4}$, em que $A\left( 0,0, \pm \frac{\pi}{4} \right) = \frac{M}{m} + \frac{1}{2}$ e $B\left( 0,0, \pm \frac{\pi}{4} \right) = \frac{1}{2}$, conduzindo a $\theta_{d}^{*} = \pm {atan}\left( \frac{2M}{m} + 1 \right)$ e uma condição de estabilidade mais flexível em que:

|                                                                                            |        |
|--------------------------------------------------------------------------------------------|--------|
| $$F \geq 0\ \ se\ \ \left| \theta_{d} \right| \leq {atan}\left( \frac{2M}{m} + 1 \right)$$ | (3.99) |

Assim, assumindo $F > 0$, as relações de estabilidade dos coeficientes do controlador (Equações 3.93, 3.94 e 3.95) podem ser resumidas a:

|                                                                           |         |
|---------------------------------------------------------------------------|---------|
| $$\frac{\lambda_{4}}{\lambda_{3}} > \frac{\lambda_{2}}{\lambda_{1}} > 0$$ | (3.100) |

Constata-se que a relação entre os coeficientes associados a $\phi$ e $\dot{\phi}$ deve ser superior à relação dos coeficientes associados ${\widetilde{y}}_{b}$ e ${\dot{\widetilde{y}}}_{b}$ e que nenhum coeficiente pode assumir valor nulo nem possuir sinal diferente dos demais.

##### Estabilidade no Deslizamento em $s_{4}$

Basicamente, aplica-se o mesmo procedimento feito para $s_{3}$, em que ${\widetilde{y}}_{b}$ é similar a ${\widetilde{x}}_{b}$ e $\phi$ é similar a $\theta$. O jacobiano (Eq. 3.87), o polinômio característico (Eq. (3.92)) e as condições de estabilidade (Equações 3.93, 3.94 e 3.95) são idênticas substituindo $\lambda_{1},\ \lambda_{2},\ \lambda_{3}$ e $\lambda_{4}$ por $\lambda_{5},\lambda_{6},\lambda_{7}$ e $\lambda_{8}$ respectivamente, e o termo $F$ pelo termo $G$, dado por:

|                                                                                                                                                                                                                                          |         |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $$G = \frac{\partial{\ddot{\widetilde{y}}}_{b}}{\partial y_{1}} = \frac{u_{1}\ m}{M(M + m)}\left\lbrack - c\phi_{d}\ C\left( \psi,\phi_{L},\theta_{L} \right) + s\phi_{d}\ D\left( \psi,\phi_{L},\theta_{L} \right) \right\rbrack$$      | (3.101) |
| $$C = \frac{M}{m} + \left\lbrack \left( s\psi c\theta_{L} + c\psi s\phi_{L}s\theta_{L} \right)^{2} + c\psi^{2}c\phi_{L}^{2} \right\rbrack$$                                                                                              | (3.102) |
| $$D = c\theta c\phi_{L}c\theta_{L}\left( c\psi s\phi_{L}c\theta_{L} + s\psi s\theta_{L} \right) + s\theta\left\lbrack s\phi_{L}s\theta_{L}c\theta_{L}\left( s\psi^{2} - c\psi^{2} \right) - c\phi_{L}^{2}s\theta_{L}^{2} \right\rbrack$$ | (3.103) |

De forma semelhante, o termo em evidência é sempre positivo para as condições de operação impostas e, no termo entre colchetes, $\phi_{d}$ dá pesos para $C$ e $D$ de forma que o módulo de $C$ é maximizado e $D$ é neutralizado para $\phi_{d} = 0$ e o oposto ocorre para $\phi_{d} = \pm \frac{\pi}{2}$. Também é possível verificar que $C \geq \frac{M}{m}$ e $|D| \leq \frac{1}{2}$. Porém, diferentemente do que ocorre para $F$, o termo estritamente positivo $C$ está multiplicado a $- \cos\phi_{d}$, que é sempre negativo para as condições de operação.

Desse modo, deseja-se fazer com que $G$ seja sempre menor do que zero condicionando o valor de $\phi_{d}$. Suspeita-se de que possa existir um valor máximo para $\phi_{d}$ que faça com que $G$ seja negativo independentemente dos valores de $C$ e $D$. De fato, o caso em que $C$ assume seu valor mínimo e $D$ assume seu valor máximo (em módulo) retrata o cenário em que $G$ assume o maior valor possível. Assim, o valor de $\phi_{d}$ que anula $G$ nestas condições extremas ($\phi_{d}^{*}$) é obtido fazendo-se $G = 0$:

|                                                                                                                                                                                                                                     |         |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $$- \cos{\left( \phi_{d}^{*} \right)\min\left( |C| \right)} + \sin\left( \phi_{d}^{*} \right)\max\left( |D| \right) = 0 \rightarrow \tan\left( \theta_{d}^{*} \right) = \pm \frac{\min\left( |C| \right)}{\max\left( |D| \right)}$$ | (3.104) |
| $$\phi_{d}^{*} = \pm {atan}\left( \frac{2M}{m} \right)$$                                                                                                                                                                            | (3.105) |

Podendo-se afirmar que:

|                                                                                      |         |
|--------------------------------------------------------------------------------------|---------|
| $$G < 0\ \ \ \ se\ \ \ \left| \phi_{d} \right| < {atan}\left( \frac{2M}{m} \right)$$ | (3.106) |

Assim, assumindo $G < 0$, as relações de estabilidade dos coeficientes do controlador (Equações 3.93, 3.94 e 3.95) resumem-se a:

|                                                                                                                                                                                                                                                                                                                |         |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $$\frac{\lambda_{8}}{\lambda_{7}} > 0\ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \  \rightarrow \ \ \ \ \ \ \ \ {sign}\left( \lambda_{8} \right) = {sign}\left( \lambda_{7} \right)$$                                                                                                                                    | (3.107) |
| $$\frac{\lambda_{5}\lambda_{8}}{\lambda_{7}^{2}} < \frac{\lambda_{6}}{\lambda_{7}} < 0\ \ \ \  \rightarrow \ \ \ \ \ \ \ {sign}\left( \lambda_{5} \right) = {sign}\left( \lambda_{6} \right) \neq {sign}\left( \lambda_{7} \right),\ \ \frac{\lambda_{8}}{\lambda_{7}} > \frac{\lambda_{6}}{\lambda_{5}} > 0$$ | (3.108) |

Em suma, constata-se que a relação entre os coeficientes associados a $\phi$ e $\dot{\phi}$ deve ser superior à relação dos coeficientes associados ${\widetilde{y}}_{b}$ e ${\dot{\widetilde{y}}}_{b}$. Adicionalmente, requer-se que os coeficientes atrelados $\phi$ e $\dot{\phi}$ tenham sinais opostos aos coeficientes atrelados a ${\widetilde{y}}_{b}$ e ${\dot{\widetilde{y}}}_{b}$.

### Resumo

<table>
<colgroup>
<col style="width: 100%" />
</colgroup>
<thead>
<tr class="header">
<th><p>Variáveis deslizantes:</p>
<table>
<colgroup>
<col style="width: 100%" />
</colgroup>
<tbody>
<tr class="odd">
<td><span class="math display"><em>s</em><sub>1</sub> = (<em>ż</em><sub><em>d</em></sub>−<em>ż</em>) + <em>λ</em><sub><em>z</em></sub>(<em>z</em><sub><em>d</em></sub>−<em>z</em>),  <em>λ</em><sub><em>z</em></sub> &gt; 0</span></td>
</tr>
<tr class="even">
<td><span class="math display"><em>s</em><sub>2</sub> = (<em>ψ̇</em><sub><em>d</em></sub>−<em>ψ̇</em>) + <em>λ</em><sub><em>ψ</em></sub>(<em>ψ</em><sub><em>d</em></sub>−<em>ψ</em>),  <em>λ</em><sub><em>ψ</em></sub> &gt; 0</span></td>
</tr>
<tr class="odd">
<td><span class="math display">$$s_{3} = \lambda_{1}{\dot{\widetilde{x}}}_{b} + \lambda_{2}{\widetilde{x}}_{b} + \lambda_{3}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) + \lambda_{4}\left( \theta_{d} - \theta \right),\ \ \frac{\lambda_{4}}{\lambda_{3}} &gt; \frac{\lambda_{2}}{\lambda_{1}} &gt; 0$$</span></td>
</tr>
<tr class="even">
<td><span class="math display">$$s_{4} = \lambda_{5}{\dot{\widetilde{y}}}_{b} + \lambda_{6}{\widetilde{y}}_{b} + \lambda_{7}\left( {\dot{\phi}}_{d} - \dot{\phi} \right) + \lambda_{8}\left( \phi_{d} - \phi \right),\ \ \frac{\lambda_{8}}{\lambda_{7}} &gt; 0,\ \ \frac{\lambda_{5}\lambda_{8}}{\lambda_{7}^{2}} &lt; \frac{\lambda_{6}}{\lambda_{7}} &lt; 0$$</span></td>
</tr>
</tbody>
</table>
<p>Variável auxiliar:</p>
<p><span class="math display">$$\left\{ \begin{array}{r}
{\widetilde{x}}_{b}^{(n)} \\
{\widetilde{y}}_{b}^{(n)}
\end{array} \right\} = \begin{bmatrix}
\cos\psi &amp; \sin\psi \\
 - \sin\psi &amp; \cos\psi
\end{bmatrix}\left\{ \begin{array}{r}
x_{d}^{(n)} - x^{(n)} \\
y_{d}^{(n)} - y^{(n)}
\end{array} \right\},\ \ \dot{\psi} = \ddot{\psi} = 0,\ \ \ \ \ n = 0,\ 1,\ 2$$</span></p>
<p>Entradas:</p>
<table>
<colgroup>
<col style="width: 100%" />
</colgroup>
<tbody>
<tr class="odd">
<td><span class="math display">$$u_{1} = \frac{{\ddot{z}}_{d} - f_{z} + \kappa_{1}s_{1} + \eta_{1}{sign}\left( s_{1} \right)}{b_{z}},\ \ \kappa_{1} &gt; 0,\ \ \eta_{1} &gt; \max\left( \left| d_{z} \right| \right),\ \ b_{z} \neq 0$$</span></td>
</tr>
<tr class="even">
<td><span class="math display">$$u_{4} = \frac{{\ddot{\psi}}_{d} - f_{\psi} + \kappa_{2}s_{2} + \eta_{2}{sign}\left( s_{2} \right)\ }{b_{\psi}},\ \ \kappa_{2} &gt; 0,\ \ \eta_{2} &gt; \max\left( \left| d_{\psi} \right| \right),\ \ b_{\psi} \neq 0$$</span></td>
</tr>
<tr class="odd">
<td><p><span class="math display">$$u_{3} = \frac{\lambda_{1}{\ddot{\widetilde{x}}}_{b} + \lambda_{2}{\dot{\widetilde{x}}}_{b} + \lambda_{3}\left( {\ddot{\theta}}_{d} - f_{\theta} \right) + \lambda_{4}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) + \kappa_{3}s_{3} + \eta_{3}{sign}\left( s_{3} \right)}{\lambda_{3}b_{\theta}},\ $$</span></p>
<p><span class="math display">                         <em>κ</em><sub>3</sub> &gt; 0,  <em>η</em><sub>3</sub> &gt; max (|<em>λ</em><sub>1</sub>+<em>λ</em><sub>3</sub>||cos<em>ψ</em><em>d</em><sub><em>x</em></sub>+sin<em>ψ</em><em>d</em><sub><em>y</em></sub>|+|<em>d</em><sub><em>θ</em></sub>|),  <em>b</em><sub><em>θ</em></sub> ≠ 0</span></p></td>
</tr>
<tr class="even">
<td><p><span class="math display">$$u_{2} = \frac{\lambda_{5}{\ddot{\widetilde{y}}}_{b} + \lambda_{6}{\dot{\widetilde{y}}}_{b} + \lambda_{7}\left( {\ddot{\phi}}_{d} - f_{\phi} \right) + \lambda_{8}\left( {\dot{\phi}}_{d} - \dot{\phi} \right) + \kappa_{4}s_{4} + \eta_{4}{sign}\left( s_{4} \right)}{\lambda_{7}b_{\phi}},$$</span></p>
<p><span class="math display">                        <em>κ</em><sub>4</sub> &gt; 0,  <em>η</em><sub>4</sub> &gt; max (|<em>λ</em><sub>5</sub>+<em>λ</em><sub>7</sub>||−sin<em>ψ</em><em>d</em><sub><em>x</em></sub>+cos<em>ψ</em><em>d</em><sub><em>y</em></sub>|+|<em>d</em><sub><em>ϕ</em></sub>|),  <em>b</em><sub><em>ϕ</em></sub> ≠ 0</span></p></td>
</tr>
</tbody>
</table></th>
</tr>
</thead>
<tbody>
</tbody>
</table>

## Simulação

Para avaliar o funcionamento do controlador, realiza-se uma simulação do sistema utilizando o software MATLAB, em que se integra a equação dinâmica descrita no Capítulo 2 com os sinais de entrada calculados pelo controlador.

Os parâmetros físicos da simulação, quando não especificados explicitamente, são adotados como mostra a Tabela 3.1:

Tabela 3.1 - Parâmetros físicos de simulação

| **Parâmetro**       | **Valor**                 | **Parâmetro**     | **Valor**         |
|---------------------|---------------------------|-------------------|-------------------|
| $$M$$               | $$2,4\ kg$$               | $$g$$             | $$9,81\ m/s^{2}$$ |
| $$m$$               | $$1,0\ kg$$               | $$c_{x},\ c_{y}$$ | $$0,2\ kg/s$$     |
| $$l$$               | $$1,0\ m$$                | $$c_{z}$$         | $$0,5\ kg/s$$     |
| $$I_{xx},\ I_{yy}$$ | $$0,055\ kg \cdot m^{2}$$ | $$c_{L}$$         | $$0,1\ kg/s$$     |
| $$I_{zz}$$          | $$0,1\ kg \cdot m^{2}$$   |                   |                   |

Os valores apresentados na Tabela 3.1 são aproximadamente as especificações do drone comercial *DJI Matrice 100*, que possui uma capacidade de carga relativamente alta, com grande potencial para a aplicação proposta (JEAONG et al., 2018).

Os parâmetros de controle, quando não especificados explicitamente, são definidos como mostra a Tabela 3.2:

Tabela 3.2 - Parâmetros de controle da simulação.

| **Parâmetro**   | **Valor** | **Parâmetro**      | **Valor** |
|-----------------|-----------|--------------------|-----------|
| $$\lambda_{z}$$ | $$5$$     | $$\lambda_{\psi}$$ | $$2$$     |
| $$\kappa_{z}$$  | $$1$$     | $$\kappa_{\psi}$$  | $$1$$     |
| $$\eta_{z}$$    | $$2$$     | $$\eta_{\psi}$$    | $$2$$     |
| $$\lambda_{1}$$ | $$2$$     | $$\lambda_{5}$$    | $$- 2$$   |
| $$\lambda_{2}$$ | $$1$$     | $$\lambda_{6}$$    | $$- 1$$   |
| $$\lambda_{3}$$ | $$5$$     | $$\lambda_{7}$$    | $$5$$     |
| $$\lambda_{4}$$ | $$0,1$$   | $$\lambda_{8}$$    | $$0,1$$   |
| $$\kappa_{1}$$  | $$1$$     | $$\kappa_{2}$$     | $$1$$     |
| $$\eta_{1}$$    | $$2$$     | $$\eta_{2}$$       | $$2$$     |

Observa-se que os parâmetros selecionados respeitam os limites especificados. Em especial, observa-se que $\frac{\lambda_{1}}{\lambda_{2}} = \frac{\lambda_{5}}{\lambda_{6}} = 2$ é menor do que $\frac{\lambda_{3}}{\lambda_{4}} = \frac{\lambda_{6}}{\lambda_{7}} = 10$ e que $\lambda_{5}$ e $\lambda_{6}$ são opostos a $\lambda_{1}$ e $\lambda_{2}$. Também se verifica que o limite de estabilidade para os ângulos de referência de atitude ($\phi_{d}^{*}$ e $\theta_{d}^{*}$) segundo as equações (3.105) e (3.97) é de aproximadamente $1.4\ rad$ ($\approx 80{^\circ}$). Os parâmetros de estabilização das variáveis deslizantes $\kappa_{z}$, $\kappa_{\psi}$, $\kappa_{1}$, $\kappa_{2}$, $\eta_{z}$, $\eta_{\psi}$, $\eta_{1}$ e $\eta_{2}$ foram dimensionados para combater o distúrbio de arrasto do modelo e perturbações internas provenientes das aproximações feitas nas equações do controlador. A fim de reduzir o impacto da descontinuidade no sinal de controle, substitui-se a função $sign(s)$ por $\tanh(50s)$.

Ressalta-se que, apesar de que uma das principais características da técnica de controle por modos deslizantes ser a robustez contra distúrbios com limiares conhecidos, este aspecto não avaliado com detalhe, pois este não é o foco da contribuição deste trabalho. Esta análise é deixada como trabalhos futuros.

Com isso, avalia-se o desempenho do controlador para entrada degrau e para condição inicial fora do equilíbrio para avaliar as condições de estabilidade deduzidas na seção anterior.

### Resposta do sistema para entrada degrau unitário

As Figuras Figura 5.6, Figura 5.7 e Figura 5.8 apresentam o comportamento dinâmico da posição, da orientação da aeronave e da orientação do cabo para quando se aplica entrada degrau unitário para a posição e ângulo de guinada da aeronave com velocidades e acelerações nulas.

<img src="media/image12.emf" style="width:4.68504in;height:3.81197in" />

Figura 3.7 – Posição e velocidade da aeronave para entrada degrau unitário.

<img src="media/image13.emf" style="width:4.68504in;height:3.72401in" />

Figura 3.8 - Orientação e velocidade angular da aeronave para entrada nula para $\phi$ e $\theta$ e degrau unitário para $\psi$.

<img src="media/image14.emf" style="width:4.59167in;height:2.48958in" />

Figura 3.9 - Orientação do cabo para entrada degrau unitário.

Na Figura 3.7 se observa que a posição da aeronave tende a convergir rapidamente para o estado desejado, apresentando erro sempre decrescente com nenhuma oscilação aparente.

Na Figura 3.8, verifica-se que os ângulos de rolagem ($\phi$) e arfagem ($\theta$) apresentam oscilações significativas, com comportamento mais agressivo no regime transiente (até aproximadamente 2 segundos depois da aplicação do sinal de entrada), passando a oscilar de forma regular com leve tendência de convergência para o ponto de equilíbrio. Por outro lado, o ângulo de guinada ($\psi$) apresenta comportamento suave semelhante ao observado para a posição da aeronave, o que era esperado, visto a condição de atuação plena em torno do eixo vertical da aeronave.

Já na Figura 3.9, observa-se que a movimentação da carga apresenta oscilação do início ao fim, mas, apesar do controlador não atuar explicitamente para conter esta oscilação, ela apresenta uma leve tendência de convergência. Dada a observação da convergência das variáveis de estado do drone, este comportamento é esperado visto o acoplamento dinâmico existente entre o estado da carga e o estado do drone, além do efeito dissipativo que o arrasto do ar provoca.

Por último, as figuras Figura 3.10 e Figura 3.10 apresentam o comportamento das variáveis deslizantes e os sinais de controle obtidos na simulação em análise.

<img src="media/image15.emf" style="width:3.56698in;height:2.67466in" />

Figura 3.10 – Comportamento das variáveis deslizantes para entrada degrau.

<img src="media/image16.emf" style="width:4.3125in;height:4.59691in" />

Figura 3.11 - Sinais de controle do CMD para entrada degrau unitário.

Observa-se na Figura 3.10 que, no momento de aplicação da entrada degrau, em que se provoca um desvio na posição do estado do sistema em relação ao ponto de equilíbrio, as variáveis deslizantes saltam para valores diferentes de zeros. Porém, imediatamente começam a decrescer, realizando a fase de aproximação até atingir o zero e entrar na fase de deslizamento. É possível perceber o efeito das duas parcelas de estabilização das variáveis deslizantes nas curvas apresentas: o de decaimento exponencial combinado com o decaimento linear, garantindo convergência rápida e em tempo finito.

Na Figura 3.11, verifica-se que o torque segue um perfil parcialmente condizente com o comportamento das variáveis do sistema. A força de propulsão $u_{1}\ $ e o torque $u_{4}$ apresentam curvas mais suaves enquanto os torques relacionados aos ângulos $\phi$ e $\theta$ são mais oscilatórios. Porém, nota-se que os sinais $u_{1}$ e $u_{4}$ apresentam um salto negativo posterior ao momento em que se provoca o desvio de referência unitário. Este momento coincide com o momento de estabilização das variáveis deslizantes. Entende-se que este efeito tem relação com a transição abrupta da variável deslizante para zero provocado pelos termos descontínuos do sinal de entrada.

Por fim, vale observar que o momento em que o sistema entra em fase deslizamento coincide com o momento em que os ângulos de rolagem e arfagem deixam de oscilar agressivamente (entre 2s e 4s). Isso significa que o comportamento observado a partir deste momento é majoritariamente definido pelas constantes $\lambda_{1}$ a $\lambda_{8}$, que, para os valores definidos, levaram à estabilização rápida da posição e a um comportamento oscilatório amortecido da atitude do drone.

### Avaliação da condição de estabilidade na superfície deslizante

Para ilustrar a validade das condições de estabilidade para os parâmetros de controle do subsistema sub-atuado, propõe-se comparar o comportamento do sistema a partir de um ponto no entorno da condição de equilíbrio para três configurações de parâmetro: uma estável, outra instável e outra na margem de estabilidade.

Para isso, toma-se como estado inicial $\phi(0) = \theta(0) = 10{^\circ}$ (aproximadamente 0.1745 radianos) e zero para todas as outras variáveis do sistema. Fixando os parâmetros de controle $\lambda_{2}$, $\lambda_{3}$, $\lambda_{4}$, $\lambda_{6}$ e $\ \lambda_{8}$ em 2 e $\lambda_{6} = - 2$, faz-se $\lambda_{5} = - \lambda_{1}$ e varia-se $\lambda_{1}$ entre 1, 2 e 4, de modo a gerar os cenários em que $\left\lbrack \frac{\lambda_{2}}{\lambda_{1}},\frac{\lambda_{6}}{\lambda_{5}} \right\rbrack > \left\lbrack \frac{\lambda_{4}}{\lambda_{3}},\frac{\lambda_{8}}{\lambda_{7}} \right\rbrack$ (instável), $\left\lbrack \frac{\lambda_{2}}{\lambda_{1}},\frac{\lambda_{6}}{\lambda_{5}} \right\rbrack = \left\lbrack \frac{\lambda_{4}}{\lambda_{3}},\frac{\lambda_{8}}{\lambda_{7}} \right\rbrack$ (marginalmente estável) e $\left\lbrack \frac{\lambda_{2}}{\lambda_{1}},\frac{\lambda_{6}}{\lambda_{5}} \right\rbrack < \left\lbrack \frac{\lambda_{4}}{\lambda_{3}},\frac{\lambda_{8}}{\lambda_{7}} \right\rbrack$ (estável) respectivamente. As Figuras Figura 3.12, Figura 3.13 e Figura 3.14 apresentam o comportamento das variáveis mais influenciadas $x$,$y$, $\phi$,$\theta$, $\phi_{L}$ e $\theta_{L}$, diretamente atreladas à dinâmica ao longo do plano $xy$.

<img src="media/image17.emf" style="width:5.90551in;height:2.43045in" />

Figura 3.12 - Comportamento do sistema no plano $xy$ para $\lambda_{1} = 1$ $\left( \left\lbrack \frac{\lambda_{2}}{\lambda_{1}},\frac{\lambda_{6}}{\lambda_{5}} \right\rbrack > \left\lbrack \frac{\lambda_{4}}{\lambda_{3}},\frac{\lambda_{8}}{\lambda_{7}} \right\rbrack \right)$.

<img src="media/image18.emf" style="width:5.90551in;height:2.43045in" />

Figura 3.13 - Comportamento do sistema no plano $xy$ para $\lambda_{1} = 2$ $\left( \left\lbrack \frac{\lambda_{2}}{\lambda_{1}},\frac{\lambda_{6}}{\lambda_{5}} \right\rbrack < \left\lbrack \frac{\lambda_{4}}{\lambda_{3}},\frac{\lambda_{8}}{\lambda_{7}} \right\rbrack \right)$.

<img src="media/image19.emf" style="width:5.90551in;height:2.41502in" />

Figura 3.14 – Comportamento do sistema no plano $xy$ para $\lambda_{1} = 4$ $\left( \left\lbrack \frac{\lambda_{2}}{\lambda_{1}},\frac{\lambda_{6}}{\lambda_{5}} \right\rbrack < \left\lbrack \frac{\lambda_{4}}{\lambda_{3}},\frac{\lambda_{8}}{\lambda_{7}} \right\rbrack \right)$.

Como esperado, no primeiro cenário (Figura 3.12) as variáveis divergem indefinidamente. Já na segunda configuração (Figura 3.13), as variáveis oscilam de forma uniforme, sem demonstrar tendência clara de convergência nem divergência. Por fim, ao ajustar as variáveis dentro das condições de estabilidade obtidas (Figura 3.14), as variáveis apresentaram comportamento claro de convergência.

4.  

# GERAÇÃO DE TRAJETÓRIAS

Embora o controlador desenvolvido leve em conta a dinâmica do drone acoplada à carga, ele se preocupa apenas em controlar a posição da aeronave, ignorando o comportamento da carga. Porém, a depender da trajetória de referência dada à aeronave, a carga pode oscilar a altas amplitudes e velocidades, de forma a degradar o desempenho do movimento como um todo.

Para diminuir este efeito, decide-se atuar na geração de trajetórias de modo a induzir a aeronave a se movimentar de forma favorável à estabilização da carga. Especificamente, explora-se a combinação de duas técnicas distintas: a primeira consiste em determinar trajetórias de referência para o drone a partir de trajetórias de posição para a carga e para o ângulo de guinada da aeronave com base na propriedade de planicidade diferencial do sistema. A segunda técnica refere-se ao i*nput shaping*, que filtra o sinal de entrada com base no conhecimento da dinâmica de vibração do sistema para gerar saídas com vibração atenuada.

## Geração de Trajetória com Base na Planicidade Diferencial do Sistema

Primeiramente, apresenta-se a técnica de geração de trajetória desenvolvida por (SREENATH; MICHAEL; KUMAR, 2013) e (MELLINGER, 2012; MELLINGER; KUMAR, 2011). Eles demonstram que o sistema se trata de um sistema diferencialmente plano de modo a ser possível determinar a posição da aeronave dadas trajetórias desejadas para a carga e para o ângulo de guinada da aeronave.

Primeiramente, apresenta-se a definição de planicidade diferencial do sistema e como ela é utilizada para gerar as trajetórias do sistema, detalhando-se a definição de cada variável do sistema. Ressalta-se que esta seção reproduz a ideia central de geração de trajetória dos trabalhos de referência[^2], diferenciando-se por adotar um sistema de coordenadas alternativo, apresentar mais detalhes nas etapas de desenvolvimento das equações e por adicionar a força de arrasto linear ao modelo.

### Planicidade Diferencial do Sistema

Dado um sistema com estado $x \in \mathbb{R}^{n}$ e entrada $u \in \mathbb{R}^{m}$, ele é dito diferencialmente plano se existe um conjunto finito de variáveis $y \in \mathbb{R}^{m}$, denominadas saídas planas, que são descritas em função do estado, da entrada e suas derivadas até uma ordem finita $p$:

|                                                    |         |
|----------------------------------------------------|---------|
| $$y = y\left( x,u,\dot{u},\ldots,u^{(p)} \right)$$ | (4.109) |

de modo que o estado e as entradas do sistema podem ser escritos como funções contínuas destas saídas e suas derivadas até uma ordem finita $q$:

|                                                  |         |
|--------------------------------------------------|---------|
| $$x = x\left( y,\dot{y},\ldots,y^{(q)} \right)$$ | (4.110) |
| $$u = u\left( y,\dot{y},\ldots,y^{(q)} \right)$$ | (4.111) |

Esta propriedade potencializa o planejamento de trajetória, pois permite que os estados desejados sejam determinados a partir de trajetórias definidas no domínio das saídas planas (FLIESS et al., 1993).

Neste contexto, é possível mostrar que o estado e as entradas do sistema drone com carga suspensa por cabo podem ser escritas em função da posição da carga e ângulo de guinada da aeronave e suas derivadas até determinada ordem. Em outras palavras, $\left\lbrack {\overrightarrow{x}}_{L},\psi \right\rbrack$ é um conjunto de saídas planas para o sistema.

### Determinação das Variáveis do Sistema

#### Determinação da Posição da Aeronave $\overrightarrow{\mathbf{r}}$ e suas Derivadas

Derivando-se a Eq. (2.20) *n* vezes e isolando o termo correspondente à posição do quadcóptero, tem-se:

|                                                                                               |         |
|-----------------------------------------------------------------------------------------------|---------|
| $${\overrightarrow{r}}^{(n)} = {\overrightarrow{r}}_{L}^{(n)} - l{\overrightarrow{p}}^{(n)}$$ | (4.112) |

Portanto, para determinar a enésima derivada de $\overrightarrow{r}$, basta ter conhecimento da enésima derivada de ${\overrightarrow{r}}_{L}$ e $\overrightarrow{p}$. O vetor $\overrightarrow{p}$ pode ser determinado em função das saídas planas a partir da equação dinâmica da carga (Eq. 2.24). Isolando-se o termo referente à tensão no cabo, tem-se:

|                                                                                                                                |         |
|--------------------------------------------------------------------------------------------------------------------------------|---------|
| $$\overrightarrow{T} = - m{\ddot{\overrightarrow{r}}}_{L} - mg{\overrightarrow{e}}_{z} - C_{L}{\dot{\overrightarrow{r}}}_{L}$$ | (4.113) |

Através da definição de $\overrightarrow{p}$, tem-se:

|                                                                                         |         |
|-----------------------------------------------------------------------------------------|---------|
| $$\overrightarrow{p} = \frac{\overrightarrow{T}}{\left\| \overrightarrow{T} \right\|}$$ | (4.114) |
| $$\left\| \overrightarrow{T} \right\| = \overrightarrow{T} \cdot \overrightarrow{p}$$   | (4.115) |

Vale observar a derivada da Eq. (4.115) não depende de $\dot{\overrightarrow{p}}$:

|                                                                                                   |         |
|---------------------------------------------------------------------------------------------------|---------|
| $$\dot{\left\| \overrightarrow{T} \right\|} = \dot{\overrightarrow{T}} \cdot \overrightarrow{p}$$ | (4.116) |

Assim, é possível determina $\dot{\overrightarrow{p}}$ na Eq. (4.114), visto que também se conhece $\dot{\overrightarrow{T}}$ pela Eq. (4.113) e as derivadas da saída plana ${\overrightarrow{r}}_{L}$. Repetindo este processo, é possível concluir que ${\overrightarrow{p}}^{(n)}$ pode ser descrito em função de ${\overrightarrow{r}}_{L}^{(n + 2)}$ e suas derivadas inferiores até ${\dot{\overrightarrow{r}}}_{L}$. Considerando que se deseja definir até $\ddot{\overrightarrow{r}}$, verifica-se pela Eq. (4.112) que é necessário calcular $\overrightarrow{p}$ até a sua segunda derivada que, por sua vez, requer o conhecimento de ${\overrightarrow{r}}_{L}$ até sua quarta derivada.

#### Determinação da Orientação $\overrightarrow{\mathbf{\eta}}$ e da força de propulsão $\mathbf{F}_{\mathbf{b}}^{\mathbf{z}}$

A equação da dinâmica da aeronave (Eq. 2.23) pode ser rearranjada da seguinte forma:

|                                                                                                                                                                                  |         |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $$M\ddot{\overrightarrow{r}} - T\overrightarrow{p} + Mg{\overrightarrow{e}}_{z} + C\dot{\overrightarrow{r}} = F_{z}^{b}{\overrightarrow{\mathbf{e}}}_{\mathbf{z}}^{\mathbf{b}}$$ | (4.117) |

Verifica-se que:

|                                                                                                                                                                                                      |         |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $$\mathbf{R}{\overrightarrow{e}}_{z} = {\overrightarrow{e}}_{z}^{b} = \frac{\overrightarrow{t}}{\left\| \overrightarrow{t} \right\|},\ \ \ \ onde\ \ \ \overrightarrow{t} = \left\{ \begin{array}{r} 
 \ddot{x} + c_{x}\dot{x} - \frac{T_{x}}{M} \\                                                                                                                                                          
 \ddot{y} + c_{y}\dot{y} - \frac{T_{y}}{M} \\                                                                                                                                                          
 \ddot{z} + g + c_{z}\dot{z} - \frac{T_{z}}{M}                                                                                                                                                         
 \end{array} \right\}$$                                                                                                                                                                                | (4.118) |

Assim, define-se um sistema de coordenadas auxiliar $\Sigma_{c} = \left\lbrack {\overrightarrow{e}}_{x}^{c},{\overrightarrow{e}}_{y}^{c},{\overrightarrow{e}}_{z}^{c} \right\rbrack$, que corresponde ao sistemas de coordenadas inercial rotacionado de $\psi$ em torno de ${\overrightarrow{e}}_{z}$, como mostra a Figura 4.1.

<img src="media/image20.emf" style="width:5.30535in;height:2.28302in" />

Figura 4.1 - Ilustração do sistema de coordenadas auxiliar $\Sigma_{c}$.

Analisando a Figura 4.1, é possível verifica que:

|                                                                                         |         |
|-----------------------------------------------------------------------------------------|---------|
| $${\overrightarrow{e}}_{y}^{c} = \left\lbrack - \sin\psi,\cos\psi,0 \right\rbrack^{T}$$ | (4.119) |

A partir de ${\overrightarrow{e}}_{y}^{c}$ é possível determinar os outros vetores unitários que compõem o sistema de coordenadas do corpo, como:

|                                                                                                                                                                                               |         |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $${\overrightarrow{e}}_{x}^{b} = \frac{{\overrightarrow{e}}_{y}^{c} \times {\overrightarrow{e}}_{z}^{b}}{\left\| {\overrightarrow{e}}_{y}^{c} \times {\overrightarrow{e}}_{z}^{b} \right\|}$$ | (4.120) |
| $${\overrightarrow{e}}_{y}^{b} = {\overrightarrow{e}}_{z}^{b} \times {\overrightarrow{e}}_{x}^{b}$$                                                                                           | (4.121) |

Assim, tem-se a matriz de rotação dada por:

|                                                                                                                                    |         |
|------------------------------------------------------------------------------------------------------------------------------------|---------|
| $$\mathbf{R} = \left\lbrack {\overrightarrow{e}}_{x}^{b},{\overrightarrow{e}}_{y}^{b},{\overrightarrow{e}}_{z}^{b} \right\rbrack$$ | (4.122) |

A partir da matriz de rotação é possível determinar os ângulos de *Euler* (SLABAUGH, 1999). Enfim, define-se $F_{z}^{b}$ substituindo $\mathbf{R}{\overrightarrow{e}}_{z}$ na Eq. (4.117).

#### Determinação da Velocidade Angular $\overrightarrow{\mathbf{\omega}}$

Derivando-se a equação de movimento da aeronave (Eq. (4.117)), tem-se que:

|                                                                                                                                                                                                                                 |         |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $$M\ \dddot{\overrightarrow{r}} + C\ddot{\overrightarrow{x}} - \dot{\overrightarrow{T}} = {\dot{F}}_{z}^{b}{\overrightarrow{e}}_{z}^{b} + F_{z}^{b}\left( \overrightarrow{\omega} \times {\overrightarrow{e}}_{z}^{b} \right)$$ | (4.123) |

Projetando esta expressão ao longo de ${\overrightarrow{e}}_{z}^{b}$, tem-se que:

|                                                                                                                                                             |         |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $$\dot{F_{z}^{b}} = \left( M\dddot{\overrightarrow{r}} + C\ddot{\overrightarrow{x}} - \dot{\overrightarrow{T}} \right) \cdot {\overrightarrow{e}}_{z}^{b}$$ | (4.124) |

Na direção perpendicular a ${\overrightarrow{e}}_{z}^{b}$ e $\overrightarrow{\omega}$, ao longo da qual se tem:

|                                                                                                                                                                                                                                                                                                    |         |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $${\overrightarrow{h}}_{\omega} = \overrightarrow{\omega} \times {\overrightarrow{e}}_{z}^{b} = \frac{1}{F_{z}^{b}}\left\{ M\dddot{\overrightarrow{r}} + C\ddot{\overrightarrow{r}} - \dot{\overrightarrow{T}} - \dot{\mathbf{F}_{\mathbf{z}}^{\mathbf{b}}}{\overrightarrow{e}}_{z}^{b} \right\}$$ | (4.125) |

Observa-se que ${\overrightarrow{h}}_{\omega}$ é a projeção de $\overrightarrow{\omega}$ no plano $x_{b}y_{b}$ rotacionada em 90°, de forma que é possível determinar as componentes da velocidade angular neste plano como$:$

|                                                                            |         |
|----------------------------------------------------------------------------|---------|
| $$p = - {\overrightarrow{h}}_{\omega} \cdot {\overrightarrow{e}}_{y}^{b}$$ | (4.126) |
| $$q = {\overrightarrow{h}}_{\omega} \cdot {\overrightarrow{e}}_{x}^{b}$$   | (4.127) |

Por fim, conhecidos $p$, $q$ e $\dot{\psi}$, a terceira componente do vetor $\overrightarrow{\omega}$ é obtida da terceira componente da Eq. (2.2):

|                                                              |         |
|--------------------------------------------------------------|---------|
| $$r = \frac{\cos\theta\dot{\psi} - \sin\phi q}{\cos\phi}\ $$ | (4.128) |

#### Determinação da Aceleração Angular $\dot{\overrightarrow{\mathbf{\omega}}}$ e Momento de Entrada ${\overrightarrow{\mathbf{\tau}}}_{\mathbf{b}}$

Para determinar $\dot{\overrightarrow{\omega}}$, aplica-se um procedimento semelhante ao que se fez para encontrar $\overrightarrow{\omega}$. Primeiramente, deriva-se a equação dinâmica mais uma vez:

|                                                                                                                                                                                                                                                                                                                                                                                                                                              |         |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $M\ \ddddot{\overrightarrow{r}} + C\dddot{\overrightarrow{r}} - \ddot{\overrightarrow{T}} = {\ddot{F}}_{z}^{b}{\overrightarrow{e}}_{z}^{b} + 2\left( \overrightarrow{\omega} \times {\dot{F}}_{z}^{b}{\overrightarrow{e}}_{z}^{b} \right) + \dot{\overrightarrow{\omega}} \times F_{z}^{b}{\overrightarrow{e}}_{z}^{b} + \overrightarrow{\omega} \times \left( \overrightarrow{\omega} \times F_{z}^{b}{\overrightarrow{e}}_{z}^{b} \right)$ | (4.129) |

Projetando esta expressão ao longo de ${\overrightarrow{e}}_{z}^{b}$, tem-se que:

|                                                                                                                                                                                                                                                                                                                       |         |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $${\ddot{F}}_{z}^{b} = \left\lbrack \left( M\ \ddddot{\overrightarrow{r}} + C\dddot{\overrightarrow{r}}\  - \ddot{\overrightarrow{T}} \right) - \overrightarrow{\omega} \times \left( \overrightarrow{\omega} \times F_{z}^{b}{\overrightarrow{e}}_{z}^{b} \right) \right\rbrack \cdot {\overrightarrow{e}}_{z}^{b}$$ | (4.130) |

Na direção perpendicular a ${\overrightarrow{e}}_{z}^{b}$ e $\dot{\overrightarrow{\omega}}$, ao longo da qual se tem:

|                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |         |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $${\overrightarrow{h}}_{\alpha} = \dot{\overrightarrow{\omega}} \times {\overrightarrow{e}}_{z}^{b} = \frac{1}{F_{z}^{b}}\left\{ M\ \ddddot{\overrightarrow{r}} + C\dddot{\overrightarrow{r}}\  - {\ddot{\mathbf{F}}}_{\mathbf{z}}^{\mathbf{b}}{\overrightarrow{e}}_{z}^{b} - 2\left( \overrightarrow{\omega} \times {\dot{F}}_{z}^{b}{\overrightarrow{e}}_{z}^{b} \right) - \overrightarrow{\omega} \times \left( \overrightarrow{\omega} \times F_{z}^{b}{\overrightarrow{e}}_{z}^{b} \right) \right\}$$ | (4.131) |

Observa-se que ${\overrightarrow{h}}_{\alpha}$ corresponde à projeção de $\dot{\overrightarrow{\omega}}$ no plano $x_{b}y_{b}$ rotacionada de 90°, de forma que é possível constatar que:

|                                                                                  |         |
|----------------------------------------------------------------------------------|---------|
| $$\dot{p} = - {\overrightarrow{h}}_{\alpha} \cdot {\overrightarrow{e}}_{y}^{b}$$ | (4.132) |
| $$\dot{q} = {\overrightarrow{h}}_{\alpha} \cdot {\overrightarrow{e}}_{x}^{b}$$   | (4.133) |

Por fim, conhecidos $\dot{p}$, $\dot{q}$ e $\ddot{\psi}$, a terceira componente do vetor $\dot{\overrightarrow{\omega}}$ é determinada a partir de terceira componente da derivada da Eq. (2.2):

|                                                                                                                                       |         |
|---------------------------------------------------------------------------------------------------------------------------------------|---------|
| $$\dot{r} = \frac{\cos\theta\ddot{\psi} - \sin\phi\dot{q} - \dot{\theta}\left( \dot{\phi} + \sin\theta\dot{\psi} \right)}{\cos\phi}$$ | (4.134) |

Ressalta-se que, para chegar à definição de $\dot{\overrightarrow{\omega}}$ e ${\overrightarrow{\tau}}_{b}$ foi necessário ter conhecimento de $\ \ddddot{\overrightarrow{r}}$ que, por sua vez, requer o conhecimento até ${\overrightarrow{r}}_{L}^{(6)}$ (vide seção 4.1.2.1). Ou seja, para determinar todas as variáveis do sistema à as acelerações angulares e torques de entrada, é necessário o conhecimento até a sexta derivada da posição da carga.

### Definição de Trajetórias para a Carga

Conhecida a planicidade diferencial do sistema, com o intuito de reduzir o balanço da carga, visa-se projetar uma trajetória estável para a carga para então obter o estado da aeronave e passá-lo como referência para o controlador. Se o controlador for capaz de seguir a referência com sucesso, espera-se que a carga cumpra sua trajetória programada.

Desse modo, projeta-se trajetórias polinomiais por partes de 13ª ordem que passam por pontos arbitrários em que as derivadas até a sexta ordem são contínuas (restrições deduzidas na seção anterior), sendo nulas nos pontos de interesse (APÊNDICE II – INTERPOLAÇÃO POLINOMIAL POR PARTES). A Figura 4.2 apresenta a posição, velocidade e aceleração de um exemplo de trajetória com estas restrições para os pontos de passagem $q_{d} = \lbrack 3,0,2\rbrack$ e tempos $t_{d} = \lbrack 0,5,10\rbrack\ s$.

<img src="media/image21.emf" style="width:3.65625in;height:3.06322in" />

Figura 4.2 - Exemplo de trajetória polinomial ponto a ponto de 13° grau com derivadas nulas até a sexta ordem para os pontos de passagem $q_{d} = \lbrack 3,0,2\rbrack$ para os tempos $t_{d} = \lbrack 0,5,10\rbrack\ s$.

Observa-se que este tipo de interpolação gera curvas suaves com paradas bem marcantes nos pontos de interesse. Definindo trajetórias dessa forma no espaço tridimensional, tem-se retas entre os pontos.

A propriedade de planicidade diferencial permite encontrar o estado do *drone* necessário para que a carga tenha o comportamento desejado, porém não há garantia de que as trajetórias resultantes sejam compatíveis com as restrições do modelo, como a de tensão do cabo sempre positiva e de ângulos de guinada e arfagem inferiores a 90°, nem que sejam factíveis de serem executadas pelo controlador.

De fato, a estrutura de geração de trajetória polinomial definida não releva características do sistema em sua formulação, podendo produzir saídas dissonantes com a dinâmica do sistema.

Uma forma natural de solucionar este problema seria tratar a definição do estado da carga como um problema de otimização que considera restrições convenientes para o estado do *drone*. Porém, este processo não se demonstra trivial (CRUZ; FIERRO, 2017; SREENATH; LEE; KUMAR, 2013).

Alternativamente, propõe-se aplicar a técnica de *input shaping*, que se destaca pela simplicidade e efetividade em reduzir a oscilação na saída do sistema por meio do embutimento de oscilações no sinal de entrada que levam a frequência natural do sistema em consideração.

## Input Shaping

### Fundamentação Teórica

A técnica de *input shaping*, foi inspirada no trabalho de operadores de guindastes experientes que conseguem manobrar cargas sem provocar balanços excessivos simplesmente pressionando o botão de acionamento repetidas vezes em momentos específicos (SINGH; SINGHOSE, 2002). O princípio deste método consiste em introduzir um ou mais sinais impulsivos na entrada do sistema para gerar uma vibração na saída que é posteriormente compensada ao introduzir outros elementos impulsivos que provocariam uma vibração contrária (QIAN; YI, 2015). A Figura 4.3 ilustra a ideia para dois sinais impulsivos.

<img src="media/image22.emf" style="width:2.98958in;height:2.35194in" />

Figura 4.3 - Ilustração de resposta de um sistema sob a ação de dois impulsos devidamente selecionados utilizando técnica de *input shaping* (Adaptado de SINGH; SINGHOSE, 2002).

Na Figura 4.3, a curva em azul é a resposta do sistema quando apenas o impulso *A<sub>1</sub>* é aplicado. A curva em vermelho consiste na resposta do sistema quando apenas o impulso *A<sub>2</sub>* é aplicado. Ambos os impulsos geram uma oscilação de mesma frequência e taxa de decaimento, comportamento este esperado para sistemas de segunda ordem com amortecimento. *A<sub>2</sub>* está dimensionado exatamente no momento de inversão do movimento causado por $A_{1}$ produzindo uma resposta oposta. Esta configuração, por sua vez, gera uma trajetória sem oscilação como ilustrado pela linha em preto.

Para obter a sequência de dois impulsos como ilustrado na Figura 4.3, parte-se da descrição da vibração residual de um sistema de frequência natural $\omega_{n}$ e fator de amortecimento $\zeta$, dada por:

|                                                                                                                   |         |
|-------------------------------------------------------------------------------------------------------------------|---------|
| $$V\left( \omega_{n},\zeta \right) = e^{- \zeta\omega\_ nt_{N}}\sqrt{C(\omega,\zeta)^{2} + S(\omega,\zeta)^{2}}$$ | (4.135) |

Em que:

|                                                                                                                          |         |
|--------------------------------------------------------------------------------------------------------------------------|---------|
| $$C\left( \omega_{n},\zeta \right) = \sum_{i = 1}^{I}{A_{i}e^{\zeta\omega_{n}t_{i}}\cos\left( \omega_{d}t_{i} \right)}$$ | (4.136) |
| $$S(\omega,\zeta) = \sum_{i = 1}^{N}{A_{i}e^{\zeta\omega_{n}t_{i}}\sin\left( \omega_{d}t_{i} \right)}$$                  | (4.137) |

$A_{i}$ e $t_{i}$ são as amplitudes e os tempos em que ocorrem os impulsos e $\omega_{d} = \omega_{n}\sqrt{1 - \zeta^{2}}$ é a frequência natural amortecida. Fazendo-se a porcentagem de vibração residual igual a zero, impondo a restrição $\sum A_{i} = 1$ e tomando a posição do primeiro impulso como $t_{1} = 0$, é possível determinar as amplitudes dos impulsos $(A_{1}$ e $A_{2}$) e o momento em que o segundo impulso ocorre : $t_{2}$ (BISGAARD; COUR-HARBO; BENDTSEN, 2008).

O filtro com dois impulsos apresentado é chamado de *zero vibration shaper* (ou *ZV shaper*). Porém, neste trabalho se opta por aplicar um modelador de três impulsos chamado *ZVD shaper* (*Zero Vibration and Derivative shaper*) em que se adiciona a restrição de derivada nula à vibração residual:

|                                                                                                                        |         |
|------------------------------------------------------------------------------------------------------------------------|---------|
| $$\frac{d}{d\omega_{n}}V\left( \omega_{n},\zeta \right) = 0,\ \ \frac{d}{d\zeta}V\left( \omega_{n},\zeta \right) = 0$$ | (4.138) |

Assim, as amplitudes e os tempos dos impulsos são dados por:

|                                                                                                                     |         |
|---------------------------------------------------------------------------------------------------------------------|---------|
| $$t_{1} = 0,\ \ t_{2} = \frac{T_{d}}{2},\ \ t_{3} = \frac{T_{d}}{3}$$                                               | (4.139) |
| $$A_{1} = \frac{1}{1 + 2K + K^{2}},\ \ A_{2} = \frac{2K}{1 + 2K + K^{2}},\ \ A_{3} = \frac{K^{2}}{1 + 2K + K^{2}}$$ | (4.140) |

Sendo que $T_{d}$ é o período amortecido e $K$ é dado por:

|                                                                    |         |
|--------------------------------------------------------------------|---------|
| $$K = \exp\left( - \frac{\zeta\pi}{\sqrt{1 - \zeta^{2}}} \right)$$ | (4.141) |

Em comparação ao ZV *shaper*, o ZVD apresenta robustez significativamente maior quanto a incertezas quanto a frequência natural do sistema. Por outro lado, introduz um atraso maior na saída do sistema, que é a tendência à medida que se considerada modeladores de ordem maior (SINGH; SINGHOSE, 2002).

### Input shaping aplicado ao problema

Levando a aplicação desta técnica para o problema proposto, primeiramente deve-se determinar a frequência natural do sistema. Para isso, toma-se a frequência natural do balanço da carga, obtida das equações dinâmicas de $\phi_{L}$ e $\theta_{L}$ linearizadas em torno do ponto de equilíbrio, em que se obtém:

|                                               |         |
|-----------------------------------------------|---------|
| $$\omega_{n} = \sqrt{\frac{(M + m)g}{Ml}\ }$$ | (4.142) |

Visto que o controlador por modos deslizantes compensa o arrasto como uma parcela de distúrbio, toma-se o coeficiente de amortecimento $\zeta$ como nulo. Assim, considerando os parâmetros físicos padrões definidos na Seção 3.3, obtém-se os valores apresentados na Tabela 4.1:

Tabela 4.1 – Parâmetros de *input shaping* para o sistema de exemplo.

| **Parâmetro**  | **Valor**         | **Parâmetro**  | **Valor**         |
|----------------|-------------------|----------------|-------------------|
| $$\omega_{n}$$ | $$3,7279\ rad/s$$ | $$\omega_{d}$$ | $$3,7279\ rad/s$$ |
| $$\zeta$$      | $$0$$             | $$T_{d}$$      | $$1,6856\ s$$     |
| $$A_{1}$$      | $$0,25$$          | $$t_{1}$$      | $$0$$             |
| $$A_{2}$$      | $$0,5$$           | $$t_{2}$$      | $$0,8428\ s$$     |
| $$A_{3}$$      | $$0,25$$          | $$t_{3}$$      | $$1,6856\ s$$     |

Assim, para transferir este comportamento para sinais de entrada arbitrários, basta fazer a convolução da sequência de impulsos projetada no sinal de entrada do sistema. Para o problema em questão, primeiramente propõe-se definir pontos e ângulos de guinada por onde se deseja que o drone passe em determinados instantes de tempo, fazer uma interpolação polinomial de restringindo velocidades e acelerações nulas nestes pontos, e então aplicar *input shaping* nas curvas resultantes da interpolação e passa-las como referência para o controlador. Neste caso, as referências para a orientação do drone são tomadas como nulas. Esta estratégia já foi utilizada com sucesso em outros trabalhos de controle de VANT’s com carga suspensa por cabo como (BISGAARD; COUR-HARBO; BENDTSEN, 2008; KLAUSEN; FOSSEN; JOHANSEN, 2017)

Adicionalmente, também se propõe combinar esta técnica com a baseada na planicidade diferencial do sistema, como será apresentado na próxima seção.

## Trajetórias Baseadas na Planicidade Diferencial do Sistema com *Input Shaping*

A Figura 4.4 ilustra a nova estratégia geração de trajetória proposta.

<img src="media/image23.emf" style="width:6.47309in;height:1.44136in" />

Figura 4.4 - Estrutura da solução final de geração de trajetória.

Como ilustrado na Figura 4.4, a estratégia final de geração de trajetória recebe os pontos de parada desejados para a posição da carga e para o ângulo de guinada do drone como entrada e então realiza uma interpolação polinomial restringindo que todas as derivadas até a sexta ordem para a posição da carga e até a segunda ordem para o ângulo de guinada sejam nulas nestes pontos. Assim, aplica-se *input shaping* nas curvas resultantes da interpolação, obtendo as saídas planas que são utilizadas para calcular o estado desejado para o drone, que é passado como referência para o controlador (Figura 3.4, Seção 3.2).

Para exemplificar o efeito do *input shaping* na saída do gerador de trajetórias, define-se uma trajetória polinomial para a carga e para o ângulo de guinada do drone que parte do ponto ${{\overrightarrow{r}}_{L}}_{i} = \lbrack 0,0, - l\rbrack$ e $\psi_{i} = 0$ até ${{\overrightarrow{r}}_{L}}_{f} = \lbrack 2,2,1\rbrack$ $\psi_{f} = \frac{\pi}{3}$ em 4 segundos, à qual se aplica *input shaping* com os parâmetros apresentados na Tabela 4.1, na Seção 4.2.2. Primeiramente, a Figura 4.5 apresenta a trajetória definida para cada componente da posição da carga, assim como da posição resultante do drone para os casos em que se aplica e quando não se aplica *input shaping* na entrada.

<img src="media/image24.emf" style="width:4.64501in;height:3.89307in" />

Figura 4.5 - Comparação entre uma trajetória polinomial ponto a ponto definida para posição da carga e a obtida para o drone segundo o modelo diferencialmente plano para as configurações em que se aplica e em que não se aplica *input shaping* na trajetória de entrada.

Na Figura 4.5, ao comparar a trajetória definida para a carga com a obtida para o drone, nota-se que esta apresenta uma leve frenagem aproximadamente na metade do tempo de subida. Em relação ao efeito do *input shaping*, observa-se que o filtro provoca uma certa inclinação das curvas, antecipando a aceleração e desaceleração do movimento no início e no final da trajetória. Também é possível verificar, principalmente no perfil da altura $z$, que o *input shaping* promove uma leve atenuação no efeito transiente. Estes efeitos também podem ser observados no espaço tridimensional, como mostra a Figura 4.6.

<img src="media/image25.png" style="width:3.82639in;height:3.26209in" />

Figura 4.6 – Visualização espacial da trajetória da carga e do drone para as configurações em que se aplica e não se aplica *input shaping* na trajetória de entrada.

A Figura 4.6 mostra que, sem aplicar *input shaping*, o drone executa uma manobra de amplitude maior, apresentando uma oscilação nítida na metade do trajeto aproximadamente. Quando se aplica o filtro, esta curva se aproxima mais a uma reta, porém é possível observar pequenas oscilações ao longo de todo o trajeto.

Por fim, vale observar o comportamento das outras variáveis do sistema. As Figuras Figura 4.7, Figura 4.8 e Figura 4.9 apresentam o perfil das variáveis que descrevem a orientação do drone, os esforços de entrada e os ângulos de orientação do cabo respectivamente.

De modo geral, nota-se que o *input shaping* promove uma redução significativa na amplitude das variáveis do sistema, com exceção do ângulo de guinada da aeronave, que segue um comportamento semelhante ao observado para a posição do drone, visto que é uma saída plana pré-definida como uma curva polinomial. Por exemplo, ao observar o primeiro gráfico da Figura 4.8, verifica-se que a força de propulsão $u_{1}$ atinge valores de até aproximadamente 45 N quando não se aplica *input shaping*, passando a assumir valores inferiores a 40 N ao aplicar o filtro. Por outro lado, verificou-se o surgimento de oscilações adicionais, porém de pouca intensidade, no comportamento dos ângulos de orientação do cabo, como mostra a Figura 4.9.

<img src="media/image27.emf" style="width:3.10586in;height:2.79248in" />

Figura 4.7 – Variáveis de estado que definem a orientação do drone obtidas por meio do modelo diferencialmente plano para quando se aplica e quando não se aplica *input shaping* em trajetória polinomial ponto a ponto definida para a posição carga.

<img src="media/image28.emf" style="width:3.54331in;height:3.88049in" />

Figura 4.8 - Sinais de controle obtidos por meio do modelo diferencialmente plano para quando se aplica e quando não se aplica *input shaping* em trajetória polinomial ponto a ponto definida para a posição carga.

<img src="media/image29.emf" style="width:3.24306in;height:2.56271in" />

Figura 4.9 - Variáveis que descrevem a orientação do cabo para quando se aplica e não se aplica *input shaping* em trajetória polinomial ponto a ponto definida para a posição carga.

5.  

# CONTROLADOR COM GERADOR DE TRAJETÓRIA

Este capítulo apresenta uma análise do desempenho do controlador combinado com as técnicas de geração de trajetórias abordadas no cumprimento do objetivo proposto.

## Estrutura da Análise

Para avaliar o desempenho da solução final de controle, propõe-se uma análise comparativa entre três configurações diferentes:

1)  Trajetórias polinomiais para o drone;

2)  Trajetórias polinomiais para o drone *input shaping*;

3)  Trajetórias polinomiais para carga *input shaping* modelo diferencialmente plano.

Define-se uma trajetória ponto a ponto comum para as configurações, considerando os pontos de referência da carga (para a configuração III) abaixo das posições de referência do drone (as configurações I e II) o correspondente ao comprimento do cabo. A Tabela 5.1 apresenta a lista de pontos de trajeto a serem passados como entrada dos interpoladores polinomiais para cada configuração, enquanto a Figura 5.1 mostra uma representação gráfica do caminho definido para a carga e para o drone.

Tabela 5.1 – Pontos de referência do interpolador polinomial para as diferentes configurações de geração de trajetórias em comparação.

| **Configuração** | **Variável**    | **Valor**                                                                            |
|------------------|-----------------|--------------------------------------------------------------------------------------|
| I, II e III      | $$x,x_{L}$$     | $$\lbrack 0,4,4,1,0\rbrack\ m$$                                                      |
| I, II e III      | $$y,\ y_{L}$$   | $$\lbrack 0,6,6,9,0\rbrack\ m$$                                                      |
| I e II           | $$z$$           | $$\lbrack 0,5,5,2,0\rbrack\ m$$                                                      |
| III              | $$z_{L}$$       | $$\lbrack - 1,\ 4,\ 4,\ 1, - 1\rbrack\ m$$                                           |
| I, II e III      | $$\psi$$        | $$\left\lbrack 0,\frac{\pi}{3},\frac{\pi}{3}, - \frac{\pi}{4},0 \right\rbrack\ rad$$ |
| I e II           | $$\phi,\theta$$ | $$\lbrack 0,0,0,0,0\rbrack\ rad$$                                                    |

<img src="media/image30.png" style="width:5.125in;height:3.87248in" />

Figura 5.1 – Trajeto de referência de teste para o drone (configurações I e II) e para a carga (configuração III)

Assume-se que estes pontos ocorrem em tempos igualmente espaçados com um tempo de espera adicional de dois segundos no início e um tempo adicional para acomodação de cinco segundos. Assim, realiza-se a simulação do sistema para cada configuração, variando-se o tempo total da manobra (tirando o tempo de espera e o de acomodação) com o intuito de verificar o desempenho do sistema para diferentes níveis de agressividade de manobra. Especificamente, considera a execução da manobra em $T = \lbrack 18,\ 15,\ 12,\ 10\rbrack\ s$. Ressalta-se que são adotados os mesmos parâmetros de simulação especificados nas Tabelas Tabela 3.1 e Tabela 3.2 da Seção 3.3.

Para comparar os resultados, define-se um conjunto de métricas calculadas sobre as variáveis de saída da simulação amostradas em intervalos 0,01 s. A Tabela 5.2 lista as métricas utilizadas para fazer a comparação.

Tabela 5.2 – Lista de métricas de comparação do comportamento do sistema controlado sob diferentes configurações de geração de trajetória.

| **Símbolo**                 | **Métrica**                                                                                                                           |
|-----------------------------|---------------------------------------------------------------------------------------------------------------------------------------|
| $$r_{RMS}$$                 | Valor eficaz do erro de posição do drone.                                                                                             |
| $$\beta_{RMS}$$             | Valor eficaz do ângulo entre o eixo vertical inercial (${\overrightarrow{e}}_{z}$) e o não inercial (${\overrightarrow{e}}_{z}^{b}$). |
| $$\alpha_{RMS}$$            | Valor eficaz do ângulo do cabo em relação à vertical durante o tempo de acomodação.                                                   |
| $${\overline{f}}_{\omega}$$ | Frequência média do módulo da frequência angular $\overrightarrow{\omega}$                                                            |

A primeira medida ($r_{RMS}$) visa quantificar o erro médio do controle de posição da aeronave. O valor $\beta_{RMS}$ busca acessar o grau de inclinação médio do *drone* ao longo de percurso. Quando $\beta_{RMS}$ é maior, significa que o *drone* apresentou ângulos de rolagem e guinada maiores e por mais tempo comparativamente. A medida $\alpha_{RMS}$ visa quantificar o grau de oscilação da carga após a execução do trajeto planejado, valendo idealmente zero. Por fim, ${\overline{f}}_{\omega}$ capta o grau de oscilação da atitude do *drone* em termos de frequência.

Ressalta-se que todas as métricas definidas são melhores à medida que são menores. O valor absoluto delas não tem muito significado. A análise é mais válida quando avaliada de forma comparativa e em conjunto com análises gráficas.

## Análise dos Resultados

A Tabela 5.3 apresenta o resultado dos cálculos dos parâmetros de análise para cada cenário de simulação.

Tabela 5.3 – Quadro comparativo de desempenho com base em métricas entre as configurações de geração de trajetória em análise para diferentes tempos de execução da trajetória de referência.

<table style="width:100%;">
<colgroup>
<col style="width: 12%" />
<col style="width: 10%" />
<col style="width: 10%" />
<col style="width: 10%" />
<col style="width: 10%" />
<col style="width: 10%" />
<col style="width: 10%" />
<col style="width: 10%" />
<col style="width: 10%" />
</colgroup>
<thead>
<tr class="header">
<th><span class="math inline"><strong>T</strong></span></th>
<th colspan="4"><strong>18 s</strong></th>
<th colspan="4"><strong>15 s</strong></th>
</tr>
</thead>
<tbody>
<tr class="odd">
<td><strong>Métrica</strong></td>
<td><p><span class="math display"><strong>r</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[m]</p></td>
<td><p><span class="math display"><strong>β</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[rad]</p></td>
<td><p><span class="math display"><strong>α</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[rad]</p></td>
<td><p><span class="math display">$${\overline{\mathbf{f}}}_{\mathbf{\omega}}$$</span></p>
<p>[hz]</p></td>
<td><p><span class="math display"><strong>r</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[m]</p></td>
<td><p><span class="math display"><strong>β</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[rad]</p></td>
<td><p><span class="math display"><strong>α</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[rad]</p></td>
<td><p><span class="math display">$${\overline{\mathbf{f}}}_{\mathbf{\omega}}$$</span></p>
<p>[hz]</p></td>
</tr>
<tr class="even">
<td>I</td>
<td>0,0235</td>
<td>0,1238</td>
<td>0,1920</td>
<td>0,1218</td>
<td>0,0290</td>
<td>0,1698</td>
<td>0,0765</td>
<td>0,1920</td>
</tr>
<tr class="odd">
<td>II</td>
<td>0,0203</td>
<td>0,0892</td>
<td>0,0186</td>
<td>0,0829</td>
<td>0,0238</td>
<td>0,1155</td>
<td>0,0230</td>
<td>0,1184</td>
</tr>
<tr class="even">
<td>III</td>
<td>0,0021</td>
<td>0,1029</td>
<td>0,0061</td>
<td>0,0653</td>
<td>0,0022</td>
<td>0,1208</td>
<td>0,0158</td>
<td>0,0788</td>
</tr>
<tr class="odd">
<td><span class="math display"><strong>T</strong></span></td>
<td colspan="4"><strong>12 s</strong></td>
<td colspan="4"><strong>10 s</strong></td>
</tr>
<tr class="even">
<td><strong>Métrica</strong></td>
<td><p><span class="math display"><strong>r</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[m]</p></td>
<td><p><span class="math display"><strong>β</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[rad]</p></td>
<td><p><span class="math display"><strong>α</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[rad]</p></td>
<td><p><span class="math display">$${\overline{\mathbf{f}}}_{\mathbf{\omega}}$$</span></p>
<p>[hz]</p></td>
<td><p><span class="math display"><strong>r</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[m]</p></td>
<td><p><span class="math display"><strong>β</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[rad]</p></td>
<td><p><span class="math display"><strong>α</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[rad]</p></td>
<td><p><span class="math display">$${\overline{\mathbf{f}}}_{\mathbf{\omega}}$$</span></p>
<p>[hz]</p></td>
</tr>
<tr class="odd">
<td>I</td>
<td>0,0404</td>
<td>0,3063</td>
<td>0,9676</td>
<td>0,3913</td>
<td>0,0514</td>
<td>0,4251</td>
<td>1,1075</td>
<td>0,3937</td>
</tr>
<tr class="even">
<td>II</td>
<td>0,0280</td>
<td>0,1525</td>
<td>0,0708</td>
<td>0,2058</td>
<td>0,0312</td>
<td>0,1860</td>
<td>0,1758</td>
<td>0,3479</td>
</tr>
<tr class="odd">
<td>III</td>
<td>0,0024</td>
<td>0,1490</td>
<td>0,0437</td>
<td>0,1916</td>
<td>0,0029</td>
<td>0,2480</td>
<td>0,0848</td>
<td>1,4683</td>
</tr>
</tbody>
</table>

Analisando os resultados da Tabela 5.3 globalmente, observa-se que a configuração I, a qual não há nenhuma estratégia explícita de redução do balanço da carga, é a que apresenta o maior erro de posição ($\alpha_{RMS}$) e intensidade de oscilação de carga em todas as simulações. Também se observa que a configuração III, que leva em consideração o modelo diferencialmente plano, apresenta o menor erro de posição e grau de da oscilação da carga em todas as simulações. Apesar das diferenças numéricas do erro de posição ($r_{RMS}$), constata-se que ele é baixo para todos os cenários (no máximo 5 cm), demonstrando a eficiência do controlador no cumprimento do seu objetivo.

Para auxiliar na continuação análise, as Figuras Figura 5.2, Figura 5.3 e Figura 5.4 apresentam o trajeto percorrido pelo *drone* e pela carga para os resultados da simulações com trajeto de 15 e de 10 segundos para as três configurações.

<img src="media/image32.png" style="width:3.05118in;height:2.91544in" /><img src="media/image34.png" style="width:3.05118in;height:2.91481in" />

Figura 5.2 - Caminho percorrido pelo drone e pela carga para a simulação de 10 e 15 segundos com a configuração I.

<img src="media/image36.png" style="width:3.05069in;height:3in" /><img src="media/image38.png" style="width:3.05118in;height:3.00821in" />

Figura 5.3 - Caminho percorrido pelo drone e pela carga para a simulação de 10 e 15 segundos com a configuração II.

<img src="media/image36.png" style="width:3.05118in;height:2.9749in" /><img src="media/image40.png" style="width:3.05118in;height:2.97151in" />

Figura 5.4 - Caminho percorrido pelo drone e pela carga para a simulação de 10 e 15 segundos com a configuração III.

Ao analisar a Figura 5.2, em que não se aplica uma estratégia especializada de geração de trajetória (configuração I), reforça-se a conclusão de que, embora o controlador seja capaz de manter a posição do drone estável, a carga balança de forma intensa, especialmente para a manobra agressiva de 10 segundos.

Embora o controlador com *input shaping* (Figura 5.3) não tenha apresentado o menor $\alpha_{RMS}$ em nenhuma das simulações, ele apresentou desempenho prático convincente neste quesito. Ao comparar com a configuração em que não se aplica *input shaping* (Figura 5.2), verifica-se que a técnica foi eficaz em promover atenuação do balanço da carga, sendo possível observar apenas leve desvio nas extremidades de parada da simulação de 10 segundos. Ressalta-se que o desvio vertical da posição carga em relação à referência observado entre os pontos de passagem é inerente do movimento da carga e que o abaulamento do trajeto do drone no segundo ponto de passagem é consequência da aplicação do *input shaping*.

Quando se aplica *input shaping* na trajetória da carga (Figura 5.4), não é possível perceber a presença de oscilações nos pontos extremos nem mesmo na simulação de 10 segundos. Nota-se um abaulamento no trajeto da carga ao passar pelo segundo ponto da trajetória na simulação de 10 segundos, mas ele ocorre devido à aplicação do *input shaping*. Também é possível observar o desvio de trajeto que o *drone* faz em relação às retas que liga os pontos de passagem, especialmente para a manobra mais agressiva.

O comportamento de atenuação da carga fica mais evidente ao analisar graficamente o estado da orientação do cabo. A Figura 5.5 apresenta o comportamento das variáveis que descrevem a orientação do cabo para a simulação de 12 segundos de trajetória com as configurações I, II e III.

<img src="media/image42.emf" style="width:2.23958in;height:3.54236in" /><img src="media/image43.emf" style="width:2.09375in;height:3.54167in" /><img src="media/image44.emf" style="width:2.13542in;height:3.54167in" />

Figura 5.5 - Estado da orientação do cabo para a simulação de 12 segundos de trajetória com as configurações I (sem *input shaping*), II (com *input shaping*) e III (com *input shaping* e modelo diferencialmente plano)

Observa-se que, depois da linha vertical pontilhada, a partir de onde se mede $\alpha_{RMS}$, o caso em que não se aplica nenhuma técnica de geração de trajetória apresenta oscilações superiores a $1\ rad$ ($\approx 57{^\circ}$). Quando se aplica *input shaping* (configuração II), o balanço é significativamente mitigado, mas ainda existente na ordem de até $0,1\ rad$ ($\approx 5,7{^\circ}$) aproximadamente. A resposta da configuração III praticamente não apresenta oscilação em regime permanente.

Para auxiliar na análise do desempenho de controle de atitude do drone, as Figuras Figura 5.6 e Figura 5.7 apresentam o comportamento das variáveis que descrevem a orientação do drone para as simulações de 15 e 10 segundos com as configurações II e III respectivamente.

<img src="media/image45.emf" style="width:2.8125in;height:4.78264in" /> <img src="media/image46.emf" style="width:2.70359in;height:4.78346in" />

Figura 5.6 - Comportamento de atitude da aeronave para as simulações de 15 e 10 segundos com a configuração II.

<img src="media/image47.emf" style="width:2.44942in;height:4.33071in" /><img src="media/image48.emf" style="width:2.47017in;height:4.33071in" />

Figura 5.7 – Comportamento de atitude da aeronave para as simulação de 15 e 10 segundos com a configuração III.

Comparando as Figuras Figura 5.6 e Figura 5.7, verifica-se que, para a simulação de 15 segundos, embora o controlador apenas com *input shaping* (Figura 5.6) receba uma referência nula para os ângulos de rolagem e arfagem, estas variáveis apresentam um comportamento semelhante ao segundo, quando se tem trajetórias bem definidas com base no modelo diferencialmente plano.

Na simulação de 10 segundos, porém, a segunda solução apresentou um comportamento mais agressivo e de amplitude, que também reflete na Tabela 5.3 por meio das métricas $\beta_{RMS}$ e ${\overline{f}}_{\omega}$. Embora tenha apresentado valores menores para estes parâmetros, a primeira solução (configuração II) apresenta um perfil ruidoso que se torna mais nítido ao observar as velocidades angulares, como mostram as Figuras Figura 5.8 e Figura 5.9.

Em ambas as configurações, o controle de guinada foi bem efetivo, apresentando leve desvio para a configuração III (Figura 5.6) da simulação de 10 segundos.

<img src="media/image45.emf" style="width:2.66667in;height:4.03472in" /><img src="media/image46.emf" style="width:2.3125in;height:4.03472in" />

Figura 5.8 – Velocidade angular do drone para as simulações de 15s e 10s com a configuração II.

<img src="media/image49.emf" style="width:2.67708in;height:4.03403in" /><img src="media/image48.emf" style="width:2.24167in;height:4.0352in" />

Figura 5.9 - Velocidade angular do drone para as simulações de 15s e 10s com a configuração III.

Por fim, vale comparar o sinal de entrada obtido para as configurações II e III. As Figuras Figura 5.10 e Figura 5.11 apresentam os esforços produzidos pelo controlador para as simulações de 15 e 10 segundos de trajetória, sendo que para a configuração III (Figura 5.11), também se apresenta os esforços ideais obtidos por meio do modelo diferencialmente plano.

<img src="media/image50.emf" style="width:3.075in;height:5.29245in" /> <img src="media/image51.emf" style="width:3.34583in;height:5.28302in" />

Figura 5.10 – Esforços de controle para as simulações de 15s e 10s com a configuração II.

<img src="media/image52.emf" style="width:3.34583in;height:5.29245in" /><img src="media/image53.emf" style="width:3.04653in;height:5.29245in" />

Figura 5.11 - Esforços de controle para as simulações de 15s e 10s com a configuração III.

De modo geral, observa-se que o torque produzido pela solução III (Figura 5.10) oscila em alta frequência em torno de valores baixos para os dois tempos de manobra, mas que chegam a atingir picos superiores aos observados no comportamento da configuração IV (Figura 5.11) para a trajetória de 15 segundos. Este comportamento influncia o comportamento da velocidade angular (Figura 5.8) e na métrica ${\overline{f}}_{\omega}$ da Tabela 5.3.

Em relação à força de propulsão, ambos os cenários apresentam um comportamento parecido para a trajetória de 15 segundos. Porém, para a trajetória de 10 segundos, o controlador com base no modelo diferencialmente plano (configuração IV) apresenta oscilações de amplitude e frequência relativamente altas (justificando o valor alto de ${\overline{f}}_{\omega}$ na Tabela 5.3). Este efeito é ainda mais intenso para os torques.

Analisando a Figura 5.11, verifica-se que os sinais obtidos pelo modelo diferencialmente plano com as calculadas pelo controlador são bem próximas, especialmente para a manobra menos agressiva. Este comportamento só não foi observado para o torque de guinada para a simulação de 10 segundos, em que o sinal gerado pelo controlador não atinge os picos previstos pelo modelo. Este desvio é coerente com a observação feita no perfil de posição e velocidade angulares de guinada mostradas nas Figuras Figura 5.7 e Figura 5.9 e este comportamento pode estar associado a uma insuficiência dos parâmetros de controle definidos, especialmente $\kappa_{\psi}$ e $\eta_{\psi}$, que estão diretamente ligados à robustez contra distúrbios externos e incertezas internas.

Em suma, verificou-se que o controlador desempenha bem o papel de controlar a posição do *drone* mesmo em condições de balanço intenso da carga, mas não contém as oscilações para trajetórias definidas arbitrariamente. Designadas a contornar este problema, as soluções de geração de trajetória avaliadas apresentaram bom desepenho no papel de atenuar a oscilação da carga, com destaque para a configuração em que se agrega a propriedade de planicidade diferencial do sistema. Porém, esta solução tendeu a degradar o trajeto de referência da aeronave a partir de certo grau de velocidade de manobra (não determinado neste trabalho, mas que pode ser explorado futuramente). A solução que aplica somente *input shaping*, por sua vez, manteve um desempenho estável em todo os aspectos (com a ressalva das componentes de alta frequência na atitude da aeronave) para todas as velocidades de manobra avaliadas.

6.  

# CONCLUSÕES

Primeiramente, o presente trabalho desenvolve um modelo dinâmico do sistema drone com carga suspensa por cabo, explicitando as equações de aceleração da aeronave e dos ângulos que descrevem a orientação do cabo de modo que possa ser utilizado pelo controlador desenvolvido.

Em seguida, apresenta-se a derivação da solução de controle por modos deslizantes (CMD) desenvolvida com foco no desafio de enfrentar o alto grau de sub-atuação do sistema com proveito do conhecimento do modelo dinâmico. Define-se um CMD clássico para controlar a altitude e ângulo de guinada da aeronave e outro que leva em conta a característica de sub-atuação da dinâmica de deslocamento horizontal da aeronave de forma explícita. Demonstra-se por meio do conceito de estabilidade de *Lyapunov* que o sistema é estável sob o efeito de distúrbios cujos limiares sejam conhecidos. Também foi demonstrado que o sistema é localmente estável no deslocamento horizontal por meio do critério de estabilidade de *Routh-Hurwitz* aplicado ao sistema em fase de deslizamento e linearizado em torno do ponto de equilíbrio, deduzindo-se restrições para as constantes de controle e valores referência de atitude. Com isso, analisou-se o comportamento de convergência do controlador para alguns cenários característicos, como nos casos em que as condições deduzidas para os parâmetros de controle não são satisfeitas.

Para reduzir o balanço da carga e, consequentemente, o seu efeito na dinâmica da aeronave, investe-se em técnicas de geração de trajetória com base na propriedade de planicidade diferencial do sistema e *input shaping*. Constatou-se que, ao definir trajetórias polinomiais ponto a ponto com derivadas nulas nos pontos passagem da carga, o modelo diferencialmente plano gera trajetórias de referência válidas para a aeronave, mas que tendem a oscilar de forma proibitiva para o controlador em manobras mais agressivas. Assim, como alternativa a soluções de otimização mais sofisticadas, propôs-se a aplicação de *input shaping* no trajeto definido para a carga, embutindo o conhecimento da dinâmica de balanço na entrada. Observou-se que esta estratégia levou a uma suavização das trajetórias resultantes do *drone*, viabilizando a realização de manobras mais agressivas.

Enfim, avaliou-se o desempenho do controlador na execução de uma trajetória ponto a ponto definida por interpolação polinomial, aplicando-se diferentes configurações de tratamento destas trajetórias e variando-se o tempo de manobra. Na primeira configuração, a curva de entrada é passada diretamente como referência para o drone, depois se aplica *input shaping* nesta curva e a repassa como referência da aeronave e, por último, aplica-se uma estratégia combinada, em que se aplica *input shaping* na trajetória definida para a carga para depois aplicar o modelo diferencialmente plano e gerar referências para a aeronave.

Constatou-se que o controlador é capaz de estabilizar a posição da aeronave em todos os casos, cumprindo o seu objetivo primário. Porém, a carga apresenta oscilações intensas quando não se trata o sinal de referência base. As soluções que aplicam *input shaping* promovem uma atenuação significativa no balanço da carga. Dentre estas soluções, a que leva em consideração o modelo diferencialmente plano desempenhou-se melhor, especialmente para manobras de baixa e média agressividade, apresentando melhor fator de atenuação, menor erro de posição e taxa de variação de atitude da aeronave. Porém, seu desempenho cai a partir de determinado limiar de velocidade de manobra, quando o sinal de referência resultante para o drone assume perfis demasiadamente agressivos para a aplicação. A solução em que se aplica *input shaping* diretamente na trajetória de base apresenta melhor resultado neste cenário. Na realidade, esta configuração é a que apresentou maior constância entre os diferentes tempos de manobra com desempenho satisfatório em relação à atenuação da carga, embora tenha apresentado componentes de alta frequência no estado de atitude da aeronave em trajetos de tempo maior.

Por fim, é importante levantar alguns pontos que ainda podem ser explorados futuramente para consolidar a solução proposta. Como já apresentado ao longo do texto, primeiramente, propõe-se levantar as condições em que a combinação de *input shaping* com o modelo diferencialmente plano passa a gerar trajetórias proibitivas e analisar a robustez do sistema, verificando o seu desempenho na presença de distúrbios externos, incerteza de parâmetros e perturbações internas. Dentre outras inúmeras oportunidades possíveis de avanço da pesquisa, sugere-se atualizar o controlador no sentido de agregar a realimentação do estado da carga a fim de promover uma atenuação de balanço ativa e não depender totalmente de soluções de malha aberta, além de explorar o problema de estimação de estados, especialmente da posição da carga, que é um dos maiores desafios desta aplicação, para então partir para testes com o sistema real.

# REFERÊNCIAS BIBLIOGRÁFICAS

**Aerial Crane & Helicopter Lifting Services**. text/html. Disponível em: \<http://www.pdgaviationservices.com/services/aerial-crane\>. Acesso em: 17 ago. 2018.

ALEXIS, K. et al. Robust Model Predictive Flight Control of Unmanned Rotorcrafts. **Journal of Intelligent & Robotic Systems**, v. 81, n. 3–4, p. 443–469, 1 mar. 2016.

ASHRAFIUON, H.; ERWIN, R. S. **Sliding control approach to underactuated multibody systems**. Proceedings of the 2004 American Control Conference. **Anais**... In: PROCEEDINGS OF THE 2004 AMERICAN CONTROL CONFERENCE. jun. 2004

ASHRAFIUON, H.; ERWIN, R. S. Sliding mode control of underactuated multibody systems and its application to shape change control. **International Journal of Control**, v. 81, n. 12, p. 1849–1858, 1 dez. 2008.

BISGAARD, M. **Modeling, Estimation, and Control of Helicopter Slung Load System**. \[s.l.\] Department of Control Engineering, Aalborg University, 2008.

BISGAARD, M.; COUR-HARBO, A. LA; BENDTSEN, J. D. **Input Shaping for Helicopter Slung Load Swing Reduction**. AIAA Guidance, Navigation and Control Conference and Exhibit. **Anais**... In: AIAA GUIDANCE, NAVIGATION AND CONTROL CONFERENCE AND EXHIBIT. Honolulu, Hawaii: American Institute of Aeronautics and Astronautics, 18 ago. 2008Disponível em: \<https://arc.aiaa.org/doi/abs/10.2514/6.2008-6964\>. Acesso em: 29 jul. 2018

BISGAARD, M.; LA COUR-HARBO, A.; DIMON BENDTSEN, J. Adaptive control system for autonomous helicopter slung load operations. **Control Engineering Practice**, Special Issue on Aerial Robotics. v. 18, n. 7, p. 800–811, 1 jul. 2010.

BOB, B. **Soldiers Train On Bambi Bucket To Fight Fires In Kosovo \| Helicopter Firefighting buckets and belly tanks \| Pinterest**. Disponível em: \<https://br.pinterest.com/pin/863213453549609958/\>. Acesso em: 17 ago. 2018.

BRESCIANI, T. **Modelling, Identification and Control of a Quadrotor Helicopter**. \[s.l: s.n.\].

CROUSAZ, C. D.; FARSHIDIAN, F.; BUCHLI, J. **Aggressive optimal control for agile flight with a slung load**. in IROS 2014 Workshop on Machine Learning in Planning and Control of Robot Motion. **Anais**...2014

CROUSAZ, C. DE et al. **Unified motion control for dynamic quadrotor maneuvers demonstrated on slung load and rotor failure tasks**. 2015 IEEE International Conference on Robotics and Automation (ICRA). **Anais**... In: 2015 IEEE INTERNATIONAL CONFERENCE ON ROBOTICS AND AUTOMATION (ICRA). maio 2015

CRUZ, P. J.; FIERRO, R. Cable-suspended load lifting by a quadrotor UAV: hybrid model, trajectory generation, and control. **Autonomous Robots**, v. 41, n. 8, p. 1629–1643, 1 dez. 2017.

DAI, S.; LEE, T.; BERNSTEIN, D. S. **Adaptive control of a quadrotor UAV transporting a cable-suspended load with unknown mass**. 53rd IEEE Conference on Decision and Control. **Anais**... In: 53RD IEEE CONFERENCE ON DECISION AND CONTROL. Los Angeles, CA, USA: IEEE, dez. 2014

DING, X. et al. A review of aerial manipulation of small-scale rotorcraft unmanned robotic systems. **Chinese Journal of Aeronautics**, 22 jun. 2018.

FAUST, A. et al. **Learning swing-free trajectories for UAVs with a suspended load**. 2013 IEEE International Conference on Robotics and Automation. **Anais**... In: 2013 IEEE INTERNATIONAL CONFERENCE ON ROBOTICS AND AUTOMATION. maio 2013

FAUST, A. et al. Automated aerial suspended cargo delivery through reinforcement learning. **Artificial Intelligence**, Special Issue on AI and Robotics. v. 247, p. 381–398, 1 jun. 2017.

FENG, Y. et al. **Adaptive controller design for generic quadrotor aircraft platform subject to slung load**. 2015 IEEE 28th Canadian Conference on Electrical and Computer Engineering (CCECE). **Anais**... In: 2015 IEEE 28TH CANADIAN CONFERENCE ON ELECTRICAL AND COMPUTER ENGINEERING (CCECE). maio 2015

FLIESS, M. et al. ON DIFFERENTIALLY FLAT NONLINEAR SYSTEMS. In: FLIESS, M. (Ed.). . **Nonlinear Control Systems Design 1992**. IFAC Symposia Series. Oxford: Pergamon, 1993. p. 159–163.

FREDDI, A.; LANZON, A.; LONGHI, S. A Feedback Linearization Approach to Fault Tolerance in Quadrotor Vehicles. **IFAC Proceedings Volumes**, 18th IFAC World Congress. v. 44, n. 1, p. 5413–5418, 1 jan. 2011.

GHAZALI, R. et al. Performance Comparison between Sliding Mode Control with PID Sliding Surface and PID Controller for an Electro-hydraulic Positioning System. **International Journal on Advanced Science, Engineering and Information Technology**, v. 1, n. 4, p. 447-452–452, 2011.

GOODARZI, F. A.; LEE, D.; LEE, T. **Geometric stabilization of a quadrotor UAV with a payload connected by flexible cable**. 2014 American Control Conference. **Anais**... In: 2014 AMERICAN CONTROL CONFERENCE. jun. 2014

GUERRERO, M. E. et al. **IDA-PBC methodology for a quadrotor UAV transporting a cable-suspended payload**. 2015 International Conference on Unmanned Aircraft Systems (ICUAS). **Anais**... In: 2015 INTERNATIONAL CONFERENCE ON UNMANNED AIRCRAFT SYSTEMS (ICUAS). Denver, CO, USA: IEEE, jun. 2015a

GUERRERO, M. E. et al. **Passivity based control for a quadrotor UAV transporting a cable-suspended payload with minimum swing**. 2015 54th IEEE Conference on Decision and Control (CDC). **Anais**... In: 2015 54TH IEEE CONFERENCE ON DECISION AND CONTROL (CDC). dez. 2015b

GUERRERO-SÁNCHEZ, M. E. et al. Swing-attenuation for a quadrotor transporting a cable-suspended payload. **ISA Transactions**, v. 68, p. 433–449, 1 maio 2017a.

GUERRERO-SÁNCHEZ, M. E. et al. Swing-attenuation for a quadrotor transporting a cable-suspended payload. **ISA Transactions**, v. 68, p. 433–449, 1 maio 2017b.

HOSSAIN, E. et al. Sliding Mode Controller and Lyapunov Redesign Controller to Improve Microgrid Stability: A Comparative Analysis with CPL Power Variation. **Energies**, v. 10, n. 12, p. 1959, dez. 2017.

JEAONG, H. et al. **Simulation and Flight Experiment of a Quadrotor Using Disturbance Observer Based Control**. 2018

KLAUSEN, K.; FOSSEN, T. I.; JOHANSEN, T. A. **Nonlinear control of a multirotor UAV with suspended load**. 2015 International Conference on Unmanned Aircraft Systems (ICUAS). **Anais**... In: 2015 INTERNATIONAL CONFERENCE ON UNMANNED AIRCRAFT SYSTEMS (ICUAS). Denver, CO, USA: IEEE, jun. 2015

KLAUSEN, K.; FOSSEN, T. I.; JOHANSEN, T. A. Nonlinear Control with Swing Damping of a Multirotor UAV with Suspended Load. **Journal of Intelligent & Robotic Systems**, v. 88, n. 2–4, p. 379–394, 1 dez. 2017.

KOTARU, P.; WU, G.; SREENATH, K. **Dynamics and control of a quadrotor with a payload suspended through an elastic cable**. 2017 American Control Conference (ACC). **Anais**... In: 2017 AMERICAN CONTROL CONFERENCE (ACC). maio 2017

KUI, Y. et al. **Sliding mode control for a quadrotor slung load system**. 2017 36th Chinese Control Conference (CCC). **Anais**... In: 2017 36TH CHINESE CONTROL CONFERENCE (CCC). jul. 2017

LEE, T.; LEOK, M.; MCCLAMROCH, N. H. **Geometric tracking control of a quadrotor UAV on SE(3)**. 49th IEEE Conference on Decision and Control (CDC). **Anais**... In: 49TH IEEE CONFERENCE ON DECISION AND CONTROL (CDC). dez. 2010

MELLINGER, D. **Trajectory generation and control for quadrotors**. PhD—\[s.l.\] University of Pennsylvania, 2012.

MELLINGER, D.; KUMAR, V. **Minimum snap trajectory generation and control for quadrotors**. 2011 IEEE International Conference on Robotics and Automation. **Anais**... In: 2011 IEEE INTERNATIONAL CONFERENCE ON ROBOTICS AND AUTOMATION. Shanghai, China: IEEE, maio 2011Disponível em: \<http://ieeexplore.ieee.org/abstract/document/5980409/\>

NISE, N. S. **Control Systems Engineering**. 6th ed ed. Hoboken, NJ: John Wiley & Sons, Incorporated, 2011.

NOTTER, S. et al. Modelling, Simulation and Flight Test of a Model Predictive Controlled Multirotor with Heavy Slung Load. **IFAC-PapersOnLine**, 20th IFAC Symposium on Automatic Control in AerospaceACA 2016. v. 49, n. 17, p. 182–187, 1 jan. 2016.

ORE, J.-P. et al. Autonomous Aerial Water Sampling. **Journal of Field Robotics**, v. 32, n. 8, p. 1095–1113, 2015.

PALUNKO, I.; FIERRO, R.; CRUZ, P. **Trajectory generation for swing-free maneuvers of a quadrotor with suspended payload: A dynamic programming approach**. 2012 IEEE International Conference on Robotics and Automation. **Anais**... In: 2012 IEEE INTERNATIONAL CONFERENCE ON ROBOTICS AND AUTOMATION. maio 2012

PERKOWSKI, M. **Helicopters cleared for Christmas tree harvest**. Disponível em: \<http://www.capitalpress.com/Oregon/20151118/helicopters-cleared-for-christmas-tree-harvest\>. Acesso em: 17 ago. 2018.

PROUTY, R. W. **Helicopter Performance, Stability, and Control**. 2002 edition ed. Malabar, Fla.: Krieger Pub Co, 2001.

QIAN, D.; YI, J. **Hierarchical Sliding Mode Control for Under-actuated Cranes: Design, Analysis and Simulation**. Berlin Heidelberg: Springer-Verlag, 2015.

RAFFO, G. V.; ALMEIDA, M. M. DE. **Nonlinear robust control of a quadrotor UAV for load transportation with swing improvement**. 2016 American Control Conference (ACC). **Anais**... In: 2016 AMERICAN CONTROL CONFERENCE (ACC). jul. 2016

RAMLI, L. et al. Control strategies for crane systems: A comprehensive review. **Mechanical Systems and Signal Processing**, v. 95, p. 1–23, 1 out. 2017.

SANKARANARAYANAN, V.; MAHINDRAKAR, A. D. Control of a Class of Underactuated Mechanical Systems Using Sliding Modes. **IEEE Transactions on Robotics**, v. 25, n. 2, p. 459–467, abr. 2009.

SHEPHERD, J.; JARVIS, A.; HUNT, T. **BC Hydro Delivers Power and Progress**. Disponível em: \<https://www.tdworld.com/transmission/bc-hydro-delivers-power-and-progress\>. Acesso em: 17 ago. 2018.

SHTESSEL, Y. et al. **Sliding Mode Control and Observation**. 2013 edition ed. New York: Birkhäuser, 2013.

SINGH, T.; SINGHOSE, W. **Input shaping/time delay control of maneuvering flexible structures**. Proceedings of the 2002 American Control Conference (IEEE Cat. No.CH37301). **Anais**... In: PROCEEDINGS OF THE 2002 AMERICAN CONTROL CONFERENCE (IEEE CAT. NO.CH37301). maio 2002

SLABAUGH, G. G. Computing Euler angles from a rotation matrix. **Retrieved on August**, v. 6, n. 2000, p. 39–63, 1999.

SPONG, M. W.; HUTCHINSON, S.; VIDYASAGAR, M. **Robot Modeling and Control**. 1 edition ed. Hoboken, NJ: Wiley, 2005.

SREENATH, K.; LEE, T.; KUMAR, V. **Geometric control and differential flatness of a quadrotor UAV with a cable-suspended load**. 52nd IEEE Conference on Decision and Control. **Anais**... In: 52ND IEEE CONFERENCE ON DECISION AND CONTROL. Florence, Italy: IEEE, dez. 2013

SREENATH, K.; MICHAEL, N.; KUMAR, V. **Trajectory generation and control of a quadrotor with a cable-suspended load - A differentially-flat hybrid system**. 2013 IEEE International Conference on Robotics and Automation. **Anais**... In: 2013 IEEE INTERNATIONAL CONFERENCE ON ROBOTICS AND AUTOMATION. maio 2013

UTKIN, V.; GULDNER, J.; SHI, J. **Sliding Mode Control in Electro-Mechanical Systems**. \[s.l.\] CRC Press, 2009.

VARGAS MORENO, A. E. **Machine learning techniques to estimate the dynamics of a slung load multirotor UAV system**. PhD—\[s.l.\] University of Glasgow, 2017.

WANG, W. et al. Design of a stable sliding-mode controller for a class of second-order underactuated systems. **IEE Proceedings - Control Theory and Applications**, v. 151, n. 6, p. 683–690, nov. 2004.

WANG, W.; LIU, X. D; YI, J. Q. Structure design of two types of sliding-mode controllers for a class of under-actuated mechanical systems. **IET Control Theory Applications**, v. 1, n. 1, p. 163–172, jan. 2007.

XIONG, J.-J.; ZHENG, E.-H. Position and attitude tracking control for a quadrotor UAV. **ISA Transactions**, v. 53, n. 3, p. 725–731, 1 maio 2014.

XU, R.; ÖZGÜNER, Ü. Sliding mode control of a class of underactuated systems. **Automatica**, v. 44, n. 1, p. 233–241, 1 jan. 2008.

ZHENG, E.-H.; XIONG, J.-J.; LUO, J.-L. Second order sliding mode control for a quadrotor UAV. **ISA Transactions**, Disturbance Estimation and Mitigation. v. 53, n. 4, p. 1350–1356, 1 jul. 2014.

ZHOU, X. et al. **Stabilization of a Quadrotor With Uncertain Suspended Load Using Sliding Mode Control**. ASME Proceedings \| 40th Mechanisms and Robotics Conference. **Anais**... In: 40TH MECHANISMS AND ROBOTICS CONFERENCE. Charlotte, North Carolina, USA: ASME, 21 ago. 2016Disponível em: \<http://dx.doi.org/10.1115/DETC2016-60060\>. Acesso em: 10 mar. 2018

ZÜRN, M. et al. **MPC controlled multirotor with suspended slung Load: System architecture and visual load detection**. 2016 IEEE Aerospace Conference. **Anais**... In: 2016 IEEE AEROSPACE CONFERENCE. mar. 2016

# APÊNDICE I – TRANSFORMAÇÕES CINEMÁTICAS

Este apêndice apresenta o procedimento de obtenção das transformações cinemáticas entre o sistema de coordenadas inercial $\Sigma_{i}$ e o não inercial $\Sigma_{b}$ representadas no texto pelas equações (2.1) e (2.2). Ressalta-se que o conteúdo desta seção é baseado em (BRESCIANI, 2008).

## Ângulos de Euler

Como informado na seção 2.1, para representar a orientação do sistema de coordenadas não inercial $\Sigma_{b}$ em relação ao referencial fixo $\Sigma_{i}$, utiliza-se a notação de Euler em que, partindo-se do referencial inercial, aplica-se três rotações consecutivas como ilustra a Figura I-1:

<img src="media/image54.emf" style="width:5.78014in;height:1.78125in" />

Figura I-1 – Sequência de rotações entre o eixo de coordenadas inercial e o não inercial.

Como ilustrado na Figura I-1, primeiramente rotaciona-se em torno do eixo z (${\overrightarrow{e}}_{z}$) de um ângulo $\psi$, resultando no sistema de coordenadas intermediário $\Sigma_{c} = \left\lbrack {\overrightarrow{e}}_{x}^{c},{\overrightarrow{e}}_{y}^{c},{\overrightarrow{e}}_{z}^{c} \right\rbrack$, sendo que ${\overrightarrow{e}}_{z}^{c} = {\overrightarrow{e}}_{z}$. Depois, rotaciona-se em torno de ${\overrightarrow{e}}_{y}^{c}$ de um ângulo $\theta$, resultando no referencial intermediário $\Sigma_{d} = \left\lbrack {\overrightarrow{e}}_{x}^{d},{\overrightarrow{e}}_{y}^{d},{\overrightarrow{e}}_{z}^{d} \right\rbrack$, ${\overrightarrow{e}}_{y}^{d} = {\overrightarrow{e}}_{y}^{c}$. Enfim, rotaciona-se em torno de sendo ${\overrightarrow{e}}_{x}^{c}$ de um ângulo $\phi$, resultando no sistema de coordenadas não inercial $\Sigma_{b} = \left\lbrack {\overrightarrow{e}}_{x}^{b},{\overrightarrow{e}}_{y}^{b},{\overrightarrow{e}}_{z}^{b} \right\rbrack$.

Assim, um vetor escrito em determinado sistema de coordenadas que esteja rotacionado em torno de um dos eixos ortogonais de outro referencial pode ser transferido para este multiplicando- o pela matriz de rotação associada àquele eixo. As equações (I-1), (I-2) e (I-3) apresentam as matrizes de rotação para os eixo $x$, $y$ e $z$ respectivamente.

|                                                     |       |
|-----------------------------------------------------|-------|
| $$\mathbf{R}_{\mathbf{x}}(\phi) = \begin{bmatrix}   
 1 & 0 & 0 \\                                         
 0 & \cos\phi & - \sin\phi \\                         
 0 & \sin\phi & \cos\phi                              
 \end{bmatrix}$$                                      | (I-1) |
| $$\mathbf{R}_{\mathbf{y}}(\theta) = \begin{bmatrix} 
 \cos\theta & 0 & \sin\theta \\                       
 0 & 1 & 0 \\                                         
  - \sin\theta & 0 & \cos\theta                       
 \end{bmatrix}$$                                      | (I-2) |
| $$\mathbf{R}_{\mathbf{z}}(\psi) = \begin{bmatrix}   
 \cos\psi & - \sin\psi & 0 \\                         
 \sin\psi & \cos\psi & 0 \\                           
 0 & 0 & 1                                            
 \end{bmatrix}$$                                      | (I-3) |

Assim, a matriz de rotação do referencial do corpo para o referencial inercial $\mathbf{R}$ apresentado no texto (Eq. (2.1)) é obtida multiplicando-se as matrizes de rotação simples na sequência definida:

|                                                                                                            |       |
|------------------------------------------------------------------------------------------------------------|-------|
| $$\mathbf{R} = \mathbf{R}_{\mathbf{z}}(\psi)\mathbf{R}_{\mathbf{y}}(\theta)\mathbf{R}_{\mathbf{x}}(\phi)$$ | (I-1) |

Ressalta-se que, para fazer a transformação inversa: do referencial inercial para o não inercial, basta fazer o mesmo cálculo com a inversa da matriz $\mathbf{R}$, que corresponde à sua transposta.

## Transformação da Velocidade Angular

A lei de controle explorada demanda a determinação da taxa de variação dos ângulos de Euler $\overrightarrow{\Omega} = \left\lbrack \dot{\phi},\dot{\theta},\dot{\psi} \right\rbrack$, mas as velocidades angulares são lidas e se manifestam nas equações dinâmicas no referencial do corpo, requerendo, portanto, uma transformação entre estas coordenadas. A matriz de rotação não se aplica a este caso, pois as taxas de variação dos ângulos de Euler não estão definidas no referencial inercial. Observa-se na figura Figura I-1 que $\dot{\psi}$ ocorre no eixo ${\overrightarrow{e}}_{z}$, $\dot{\theta}$ ocorre no eixo ${\overrightarrow{e}}_{y}^{c}$ e $\dot{\phi}$ acontece no eixo ${\overrightarrow{e}}_{x}^{b}$. Assim, a matriz de transformação demandada é obtida como mostra a Eq. (I-1):

|                                                                                 |       |
|---------------------------------------------------------------------------------|-------|
| $$\left\{ \begin{array}{r}                                                      
 p \\                                                                             
 q \\                                                                             
 r                                                                                
 \end{array} \right\} = \left\{ \begin{array}{r}                                  
 \dot{\phi} \\                                                                    
 0 \\                                                                             
 0                                                                                
 \end{array} \right\} + R_{x}^{T}(\phi)\left\{ \begin{array}{r}                   
 0 \\                                                                             
 \dot{\theta} \\                                                                  
 0                                                                                
 \end{array} \right\} + R_{x}^{T}(\phi)R_{y}^{T}(\theta)\left\{ \begin{array}{r}  
 0 \\                                                                             
 0 \\                                                                             
 \dot{\psi}                                                                       
 \end{array} \right\}$$                                                           | (I-1) |

Obtendo-se:

|                                        |       |
|----------------------------------------|-------|
| $$\left\{ \begin{array}{r}             
 p \\                                    
 q \\                                    
 r                                       
 \end{array} \right\} = \begin{bmatrix}  
 1 & 0 & - \sin\theta \\                 
 0 & \cos\phi & \sin\phi\cos\theta \\    
 0 & - \sin\phi & \cos\phi\cos\theta     
 \end{bmatrix}\left\{ \begin{array}{r}   
 \dot{\phi} \\                           
 \dot{\theta} \\                         
 \dot{\psi}                              
 \end{array} \right\}$$                  | (I-2) |

Observa-se que a Eq. (I-2) corresponde à relação inversa da Eq. (2.2).

# APÊNDICE II – INTERPOLAÇÃO POLINOMIAL POR PARTES

Esta seção apresenta a técnica de interpolação polinomial por partes, comumente chamada de *spline*, utilizada na solução de planejamento de trajetórias deste trabalho. Primeiramente, ilustra-se o conceito para polinômios de 3ª ordem com velocidade contínua e depois se estende a análise para polinômios de ordem maior. Ressalta-se que o conteúdo desta seção é baseado em (SPONG; HUTCHINSON; VIDYASAGAR, 2005).

O procedimento parte da definição da posição entre dois pontos $q\left( t_{0} \right)$ e $q\left( t_{f} \right)$ como um polinômio de terceiro grau em função do tempo:

|                                                     |        |
|-----------------------------------------------------|--------|
| $$q(t) = a_{0} + a_{1}t + a_{2}t^{2} + a_{3}t^{3}$$ | (II-1) |

Derivando-se a Eq. (II-II-1), obtém-se a expressão para a velocidade:

|                                          |        |
|------------------------------------------|--------|
| $$v(t) = a_{1} + 2a_{2}t + 3a_{3}t^{2}$$ | (II-2) |

Observa-se que existem quatro coeficientes desconhecidos: $a_{0}$, $a_{1}$, $a_{2}$ e $a_{3}$; necessitando de quatro restrições para defini-las. Além da especificação das posições final e inicial, determina-se valores desejados para as velocidades nestes pontos, de modo que:

|                                                                  |        |
|------------------------------------------------------------------|--------|
| $$q_{0} = a_{0} + a_{1}t_{0} + a_{2}t_{0}^{2} + a_{3}t_{0}^{3}$$ | (II-2) |
| $$q_{f} = a_{0} + a_{1}t_{f} + a_{2}t_{f}^{2} + a_{3}t_{f}^{3}$$ | (II-3) |
| $$v_{0} = a_{1} + 2a_{2}t_{0} + 3a_{3}t_{0}^{2}$$                | (II-4) |
| $$v_{f} = a_{1} + 2a_{2}t_{f} + 3a_{3}t_{f}^{2}$$                | (II-5) |

As Equações II-2 a II-5 formam um sistema linear (Eq. II-6) do qual é possível obter os valores dos coeficientes:

|                                                 |        |
|-------------------------------------------------|--------|
| $$\begin{bmatrix}                               
 1 & t_{0} & t_{0}^{2} & t_{0}^{3} \\             
 0 & 1 & 2t_{0} & 3t_{0}^{2} \\                   
 1 & t_{f} & t_{f}^{2} & t_{f}^{3} \\             
 0 & 1 & 2t_{f} & 3t_{f}^{2}                      
 \end{bmatrix}\left\{ \begin{array}{r}            
 a_{0} \\                                         
 a_{1} \\                                         
 a_{2} \\                                         
 a_{3}                                            
 \end{array} \right\} = \left\{ \begin{array}{r}  
 q_{0} \\                                         
 v_{0} \\                                         
 q_{f} \\                                         
 v_{f}                                            
 \end{array} \right\}$$                           | (II-6) |

A Figura II-1 apresenta uma curva gerada por este método para os tempos $\left\lbrack t_{0},t_{f} \right\rbrack = \lbrack 0,4\rbrack$ e restrições $\left\lbrack q_{0},q_{f},v_{0},v_{f} \right\rbrack = \lbrack 0,4,0,0\rbrack$.

<img src="media/image55.emf" style="width:3.69792in;height:2.77178in" />

Figura II-1 – Exemplo de trajetória polinomial cúbica com restrições de posição e velocidade nos pontos extremos.

Para definir uma trajetória com múltiplos pontos de passagem, basta gerar polinômios em partes de modo que a posição e velocidade inicial de uma parte coincida com a posição e velocidade final da curva anterior.

Para estender o conceito para polinômios de ordem maior, basta identificar o padrão da solução. Verifica-se que ao especificar a posição e a velocidade dos dois pontos, completa-se 4 restrições que solucionam a obtenção dos 4 coeficientes do polinômio que determina a posição ao longo do tempo. Assim, para cada diferenciação acrescida, aumenta-se o número de restrições e a ordem do polinômio da posição em dois. Sendo $p$ a ordem do polinômio da posição e $d$ a ordem da maior derivada especificada nos pontos de interesse, tem-se que $p = 2d + 1$. Portanto trajetórias com especificação até a sexta derivada são obtidas polinômios de 13ª ordem.

[^1]: Alternativamente, pode-se substituir a Eq. (2.24) na Eq. (2.23) e resultar em uma equação em função da posição da carga ${\overrightarrow{r}}_{L}$. O resultado é semelhante à Eq. (2.24).

[^2]: No de diz respeito a geração de trajetória, (SREENATH; MICHAEL; KUMAR, 2013) também considera momentos em que a tensão no cabo é nula, desenvolvendo um modelo dinâmico híbrido. O presente trabalho faz referência apenas à análise do modelo para quando a tensão no cabo não é nula.
