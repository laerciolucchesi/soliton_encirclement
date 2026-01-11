Ideias e direções de pesquisa associadas ao projeto encirclement soliton-inspired controllers

(Discussões após reunião com Markus e Bruno — 08/jan/2026)



**1.Mapeamento da literatura relacionada ao controle tangencial soliton-inspired:** Investigar de forma sistemática se já existem propostas equivalentes ou conceitualmente próximas ao controle tangencial soliton-inspired, não se restringindo ao problema de encirclement, mas abrangendo outras aplicações de controle distribuído, sincronização, ondas viajantes, cadeias acopladas e sistemas inspirados em solitons.

**2.Levantamento do estado da arte em encirclement:** Identificar e organizar as principais abordagens existentes para o problema de encirclement, incluindo trabalhos fundacionais e soluções mais recentes, destacando hipóteses, modelos de comunicação, requisitos de informação global/local e métricas de desempenho utilizadas.

**3.Implementação de uma lei de controle soliton que reproduza a dinâmica da KdV:** Investigar e implementar uma formulação de controle soliton-inspired cuja evolução coletiva aproxime (ou recupere, em um limite contínuo) a equação de Korteweg–de Vries (KdV), buscando reproduzir propriedades típicas de solitons — como propagação de ondas viajantes estáveis e interações do tipo “quase elásticas” — e avaliar se essa estrutura melhora estabilidade, coesão e robustez do encirclement sob perturbações e imperfeições de comunicação.

**4.Robustez a ruído de sensoriamento:** Introduzir ruído nas medições locais (posição, ângulo, distância ao alvo e aos vizinhos) para avaliar a robustez do controlador soliton-inspired diante de incertezas realistas de percepção.

**5.Impacto de imperfeições de comunicação:** Avaliar o desempenho do controlador sob diferentes condições adversas de comunicação, incluindo limitação de alcance, atrasos variáveis e falhas intermitentes de comunicação entre agentes.

**6.Sensibilidade ao período de controle:** Investigar como diferentes períodos de atualização do controle afetam a estabilidade, a convergência e a qualidade da formação, identificando limites operacionais e regimes estáveis.

**7.Exploração de funções não lineares alternativas ao termo cúbico:** Analisar o efeito de substituir o termo u^3  por outras funções ímpares não lineares (por exemplo, tan(u) ou variantes suavizadas),  comparando propriedades de estabilidade, saturação, dissipação e comportamento transitório.

**8.Extensão para controle feedforward opcional:** Propor e avaliar a inclusão de um termo feedforward complementar ao controle feedback atual, visando melhorar resposta dinâmica, antecipação de movimento do alvo ou compensação de perturbações sistemáticas.

**9.Influência de falhas de agentes na formação:** Estudar explicitamente o impacto de falhas temporárias ou permanentes de drones na estabilidade da formação e nas métricas globais, reforçando a análise de resiliência do controlador.

**10.Impacto de entrada e saída dinâmica de agentes:** Avaliar o comportamento da formação quando drones entram ou saem dinamicamente do enxame (além de falhas abruptas), verificando se o mecanismo de acoplamento local permite reconfiguração suave.

**11.Escalabilidade em função do número de agentes:** Avaliar como as métricas de desempenho, convergência e robustez variam com o número de drones na formação, verificando propriedades de escalabilidade e de independência do conhecimento global do tamanho do enxame.

**12.Levantamento de parâmetros dinâmicos típicos na literatura:** Identificar e sistematizar valores típicos de parâmetros que caracterizam a dinâmica de diferentes classes de drones, com base na literatura, considerando ao menos três perfis representativos — por exemplo, drones de corrida, drones de vigilância/cinematografia e drones de carga — a fim de fundamentar cenários realistas para simulações e análises comparativas.

**13.Desempenho sob dinâmicas heterogêneas dos agentes:** Investigar o comportamento do controlador soliton-inspired quando aplicado a enxames de drones com dinâmicas distintas (por exemplo, diferentes limites de velocidade, aceleração, atraso de atuação ou modelos de movimento), avaliando os impactos na coesão, na sincronização e na qualidade da formação.

**14.Generalização para outras formações geométricas:** Investigar a adaptação do arcabouço soliton-inspired para outras formações, em particular a formação em “V”, analisando se os princípios de acoplamento local e propagação tipo onda permanecem válidos.

**15.Inclusão de perturbações externas no modelo dinâmico:** Estender o modelo para incorporar ações de perturbação externa, como rajadas de vento, correntes persistentes ou forças ambientais exógenas, avaliando o impacto dessas perturbações na estabilidade, coesão e qualidade da formação, bem como a capacidade do controlador soliton-inspired de rejeitar distúrbios e recuperar o regime estacionário.

**16.Regras de projeto do controlador e análise conjunta de estabilidade:** Propor regras sistemáticas de projeto para o controlador soliton-inspired, estabelecendo relações claras entre os parâmetros do controle, as dinâmicas dos agentes e as condições de comunicação, e analisar conjuntamente as implicações dessas escolhas sobre a estabilidade, convergência e robustez do sistema, fornecendo diretrizes práticas para configuração segura e eficaz do controlador.

**17. Extensão para encirclement em 3D e levantamento da literatura correspondente:** Propor uma solução para encirclement em três dimensões, adaptando o arcabouço soliton-inspired para formações 3D (por exemplo, órbitas em torno do alvo em um plano móvel, circundamento em “cinturão” com controle de altitude, ou formações sobre uma superfície esférica), e verificar na literatura como o encirclement 3D é modelado e resolvido, identificando hipóteses usuais, requisitos de sensoriamento/comunicação e métricas específicas de qualidade da formação em 3D.

**18. Encirclement sem GPS com base apenas em comunicação e medições relativas:** Analisar a viabilidade de executar o controlador soliton-inspired em um cenário GPS-denied, utilizando somente informação relativa obtida via comunicação entre drones (por exemplo, trocas de distâncias, velocidades relativas, estimativas de rumo/bearing, e/ou odometria local), e avaliar quais variáveis mínimas são necessárias para manter estabilidade, espaçamento uniforme e rastreamento do alvo, incluindo o impacto de ruído e atrasos nessas medições relativas.



