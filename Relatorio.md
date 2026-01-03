# ***Projeto de CG - Inverse Kinematics***

![Planar Ik](Imagens_Videos/Planar_ik.gif)
## **Autor:**

Rafael Canhoto a22401658

## **Pesquisa**

Este projeto consiste em criar um sistema de inverse kinematics e aprofundar os meus conhecimentos sobre o mesmo, vou usar o unity para o realizar.

### **O que é inverse kinematics?**

![FK vs IK](Imagens_Videos/FK_vs_IK.gif)

Inverse kinematics é o estudo do movimento de um objeto sem ter em conta o torque e forças aplicadas ao mesmo.
Ou seja, é um conjunto de calculos das variáveis das juntas e dos conectores a um ponto final no objeto (PFO). Dando a posição e a orientação do PFO conseguimos calcular as todas as variaveis consoante as juntas e os conectores incluindo posição, rotação e orientação.

![ ](Imagens_Videos/InverseKinematics_1.jpg)

### **Como calcular?**

Existem duas formas principais de calcular as inverse kinematics, uma é a forma analitica e a outra é a forma numérica.

Ambas têm as suas vantagens e desvantagens.

***Vantagens nas formas numericas:***

- Versatil e geral, consegue lidar com robos/braços que tenham muitas juntas e limites complexos.
- Implementação é simples;
- Facil de adicionar vários limites através de otimizações;

***Desvantagens nas formas numericas:***

- Lento a encontrar uma solução pois precisa de iterar mais vezes, afetando a performance em tempo real;
- A qualidade e velocidade da solução dependem muito da configuração das juntas;
- Pode ser pesado para calcular se tiver um braço extremamente complexo ou se precisar de muita precisão;

***Vantagens forma analitica:***

- Computam todas as soluções existindo ou não;
- Depois de todas as equações serem derivadas torna-se rápido de encontrar as outras soluções;
- Não é preciso definir parametros;

***Desvantagens forma analitica:***

- Normalmente são dificeis ou "chatas" de derivar;
- Tem que se derivar para cada robot individualmente;
- Apenas são aplicadas a robos com um objetivo fixo;

---

### Formas Numéricas

#### **Jacobian ik**

Método que usa matrizes jacobianas para calcular o movimento das juntas e dos conectores. Uma matriz jacobiana é uma matriz que usa a primeira derivada de uma função valor de um vector.

Ou seja, depois de darmos um "objetivo" para o PFO conseguimos calcular qual será o angulo que a/as juntas têm que fazer para conseguir chegar lá, consequentemente as conexões acompanham as juntas.

![ ](Imagens_Videos/Jacobian.jpg)

Alguma matemática. Primeiro temos que calcular a matriz jacobiana para conseguirmos fazer as iterações sobre as juntas.

![Função da Matriz jacobiana](Imagens_Videos/JacobianMatrixFunction.png)

O ***J*** pode ser visto como ***k x n*** em que os valores sao vetores IR^3 ou como ***m x n*** onde os valores são escalares (m = 3k).

A primeira equação para descrever as velocidades do PFO pode ser escrita da seguinte forma, sendo que usamos as primeiras derivadas.

![Dot notation](Imagens_Videos/DotNotation.png)

Depois a começa a iteração sobre estas equações para descobrir uma solução.

Suponhamos que temos o valor dos angulos (θ), do PFO (vector s) e do alvo(t), com isto matriz pode ser computada ***J = J(θ)***. Depois procuramos atualizar o angulo para conseguirmos "modificar" o angulo da junta Δθ:

![Update angles](Imagens_Videos/UpdateAgles.png)

As mudanças ao PFO podem ser vistas na formula seguinte

![End effector changes](Imagens_Videos/EndEffectorChanges.png)

A jacobiana é um calculo muito sensivel a pequenas alterações nos angulos das juntas e isto deixa-nos com dois problemas:

- se o alvo estiver muito distante os braços acabam por esticar para tentar conseguir chegar ao alvo;
- por outro lado se movermos o alvo muito perto do braço e com alterações pequenas o braço começa a ter "tremeliques", pois está constantemente a tentar chegar ao alvo sem sucesso.
  
Este efeito pode ser reduzido com algoritmos DLS e SDLS, mas é dificil de retira-los por completo.

> Depth Limited Search (DLS) é uma variante da Depth first Search (DFP), algoritmo transversal.

Agora que já temos uma matriz vamos adicionar alguns limites nas juntas para o nosso braço não rodar para onde não queremos/é suposto.

**Pole vector** é um desses limites usado em braços com mais do que 1 junta. Isto limita a rotação de uma das juntas num planos, normalmente usado em cotovelos ou joelhos para não dobrarem na direção oposta e o modelo humano não fazer rotações esquisitas ou ficar com joelhos de galinha.

![Pole Vector](Imagens_Videos/Pole_Vector.gif)

#### **Cyclic Coordinates Descent (CCD)**

A ideia por detrás da CCD é, se ***f*** é uma função dimensional de ***k***, então conseguimos minimizar sucessivamente o ***f*** por dimensão individual ciclicamente conseguindo manter o ***f*** nas outras dimensões.

A vantagem deste algoritmo é que pega numa função complexa de ***k*** e reduz para uma função simples e unica de ***k***.

A desvantagem é que pode ser lenta principalmente se o ***f*** não for bem instruido.

Ou seja, CCD resolve o problema através de otimização. Fazendo um loop nas juntas do final até ao inicio mantendo o PFO mais junto possivel do alvo.
O loop só acaba quando conseguir chegar ao alvo ou quando chegar ao limite de iterações dadas.

![CCD](Imagens_Videos/CCD.jpg)

#### **Forward And Backward Reaching Inverse Kinematics (FABRIK)**

FABRIK é uma solução heuristica para a ik, o que isto quer dizer é que procura uma postura correta aplicando um conjunto de regras para transformar a postura inicial na final.

FABRIK originalmente foi pensado ser rapido e leve para soluções de inverse kinematics aplicadas a computação grafica, mas tem sido usada em muitas mais areas para alem dessa, como por exemplo animações de humanos em tempo real ou gravadas.

Recentemente também tem sido usado na robotica para posições fixas ou moveis com várias juntas.

Uma vantagem desta solução é que precisa de muito pouca informação para além da estrutura da ik, ou seja, comprimento da ligação das juntas, posição relativa da junta, tipo de alcance e os limites da mesma.

Tem uma desvantagem que FABRIK não garante uma solução viável, mas em desenvolvimento de jogos pode fornecer boas soluções na grande maioria dos casos (isto referindo a movimentos de humanos).

> Aristidou et al., 2016; Aristidou & Lasenby, 2011; M. Santos et al., 2021; Tao et al., 2021

A solução de FABRIK funciona no espaço e posição das juntas e é aplicado hierarquicamente a cada junta e é iterado até ser encontrada uma solução.\
Cada movimento é feito através da distancia mais pequena possivel para conseguir reposicionar a junta anterior no braço mantendo os limites do sistema.

![Fabric](Imagens_Videos/FABRIK.png)

---

### Forma analitica

A forma analitica é normalmente usada em sistemas simples e pequenos, pois usa soluções fechadas que fornecem um resultado mais instantaneo. Usa uma abordagem geometrica utilizando trigonometria e vetores para conseguir derivar as juntas.\
Para obter essas soluções fechadas calcula várias equações e variaveis ao mesmo tempo.

---

## **Técnicas usadas**

Jacobian ik

Joint constraints

Pole vector

---

## **Técnicas implementadas**

Jacobian 1 eixo

Jacobian 2 eixos

Jacobian 2 eixos + rodar o cotovelo

Jacobian ik com constraints

Pole vector

---

## **Problemas/erros/obstaculos durante o trabalho**

Antes de começar o projeto, não me organizei bem com o tempo e não fiz uma boa gestão entre djd2 e cg. O que me dificultou bem mais o trabalho, por isso investi mais na investigação.

Quando comecei, tive alguns problemas na atribuição de juntas pois estava a mete-las pela ordem errada e isso fazia com que o código nao funcionasse.
Depois o meu target não parava no sitio (erro comum, tinha o target dentro do end effector, ou seja quando o end effector mexia o target também, logo nunca chegava ao destino marcado).

Depois não estava a conseguir fazer com que o cotovelo rodasse se quer.

Tive alguns problemas na implementação das constraints.

Alguns problemas com o pole vector, pois quando o implementei deixei de conseguir mexer o braço para cima. E apesar de perceber a funcionalidade de um pole vector, a sua função prática parecia ir contra a definição então acho que não ficou bem implementado.\
(tenho um pequeno video na pasta das imagens a mostrar com o braço mexia antes de aplicar o pole vector.)

Também tive alguns problemas na investigação pois quase todos os estudos levavam para a robotica e não necessariamente para jogos o que achei estranho visto que grande parte dos jogos usam ik, entao acabei por focar mais nesse ambito.\
Mais tarde comecei a ficar um bocado confuso com os vários metodos pois pareciam dizer todos a mesma coisa, mas de formas diferentes o que acabou por me causar alguma confusão.

---

## **Conclusão**

Apesar de ter feito uma má gestão do meu tempo, naquele que tive penso que consegui aprender bastante sobre inverse kinematics. Tem muita mais matemática do que estava à espera, todas as formas que encontrei por mais distintas que sejam acabavam por levar todas ao mesmo objetivo. Conseguir mover um objeto/braço/robo com juntas e ligações entre elas. Ambos têm as suas vantagem e desvantagens daí serem usados metodos diferentes consoante o objetivo que se quer alcançar, com que finalidade, precisão e rapidez./
Nos jogos acaba por ser usado para "mostrar" um bocado mais de realismo (nem sempre pode ser usado para fazer o contrário) e parecer mesmo que o personagem está no mundo onde está inserido.

![InGame](Imagens_Videos/In_Game.gif)

---

## **Bibliografia**

[Forward kinematics vs inverse kinematics](https://irisdynamics.com/articles/forward-and-inverse-kinematics)

[What is inverse kinematics](https://www.mathworks.com/discovery/inverse-kinematics.html)

[What is inverse kinematics 2.0](https://fiveable.me/robotics/unit-2/inverse-kinematics-analytical-numerical-methods/study-guide/PrxpB2XwD4YEgi1u)

[Inverse Kinematics](https://motion.cs.illinois.edu/RoboticSystems/InverseKinematics.html)

[Jacobian ik](https://medium.com/unity3danimation/overview-of-jacobian-ik-a33939639ab2)

[Jacobian ik 2.0](https://medium.com/unity3danimation/analytical-jacobian-ik-cb3df86edf00)

[Jacobian ik math](https://www.cs.cmu.edu/~15464-s13/lectures/lecture6/iksurvey.pdf)

[Jacobian ik math 2.0](https://roboticsnakamura.wordpress.com/wp-content/uploads/2020/06/advanced-robotics-y.-nakamura.pdf)

[Unity transform](https://docs.unity3d.com/6000.3/Documentation/ScriptReference/Transform.html)

[Unity ik](https://docs.unity3d.com/es/current/Manual/InverseKinematics.html)

[Jacobian study](https://mtsu.pressbooks.pub/robotics/chapter/chapter-4/)

[Video Jacobian explanation](https://www.youtube.com/watch?v=2_cdDGwnl80)

[Waht is DLS](https://www.educative.io/answers/what-is-depth-limited-search)

[Coordinate descent](https://bookdown.org/rdpeng/advstatcomp/coordinate-descent.html)

[CCD 2D](https://www.ryanjuckett.com/cyclic-coordinate-descent-in-2d/)

[FABRIK](https://pubs.lib.uiowa.edu/dhm/article/31772/galley/140227/view/)

[Pole VEctor](https://mykolbe.wordpress.com/2020/09/03/rig-fundamentals-polevector/)

[Video Inverse kinematics](https://www.youtube.com/watch?v=jfKTmEWJESwhttps://www.youtube.com/watch?v=jfKTmEWJESw)
