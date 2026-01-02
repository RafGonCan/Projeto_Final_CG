# ***Inverse Kinematics***

## **Pesquisa**

### **O que é inverse kinematics?**

Inverse kinematics é o estudo do movimento de um objeto sem ter em conta o torque e forças aplicadas ao mesmo.
Ou seja, é um conjunto de calculos das variáveis das juntas e dos conectores a um ponto final no objeto (PFO). Dando a posição e a orientação do PFO conseguimos calcular as todas as variaveis consoante as juntas e os conectores incluindo posição, rotação e orientação.

![ ](Imagens_Videos/InverseKinematics_1.jpg)

### **Como calcular?**

Existem duas formas principais de calcular as inverse kinematics, uma é a forma analitica e a outra é a forma numérica.

A forma numérica pode ser usada de várias formas, como por exemplo:

- **Jacobian ik**;

- **Cyclic Coordinate Descent (CCD)**;

- **Forward And Backward Reaching Inverse Kinematics (FABRIK)**;

- **Newton-Raphson/Levenberg-Marquardt**;

A forma analitica

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

A solução de FABRIK funciona no espaço e posição das untas e é aplicado hierarquicamente a cada junta e é iterado até ser encontrada uma solução.\
Cada movimento feito é através da distancia mais pequena possivel para conseguir reposicionar a junta anterior no braço mantendo os limites do sistema.

---

### Formas analiticas

---

## **Técnicas usadas**

Jacobian ik

Joint constraints

---

## **Técnicas implementadas**

Jacobian ik com constraints

---

## **Problemas/erros que tive durante o trabalho**

Antes de começar o projeto, não me organizei bem com o tempo e não fiz uma boa gestão entre djd2 e cg. O que me dificultou bem mais o trabalho, por isso investi mais na investigação.

Quando comecei, tive alguns problemas na atribuição de juntas pois estava a mete-las pela ordem errada e isso fazia com que o código nao funcionasse.
Depois o meu target não parava no sitio (erro comum, tinha o target dentro do end effector, ou seja quando o end effector mexia o target também logo nunca chegava ao destino marcado).

Depois não estava a conseguir fazer com que o cotovelo rodasse se quer.

Tive alguns problemas na implementação das constraints.

---

## **Bibliografia**

[Forward kinematics vs inverse kinematics](https://irisdynamics.com/articles/forward-and-inverse-kinematics)

[What is inverse kinematics](https://www.mathworks.com/discovery/inverse-kinematics.html)

[Jacobian ik](https://medium.com/unity3danimation/overview-of-jacobian-ik-a33939639ab2)

[Jacobian ik 2.0](https://medium.com/unity3danimation/analytical-jacobian-ik-cb3df86edf00)

[Jacobian ik math](https://www.cs.cmu.edu/~15464-s13/lectures/lecture6/iksurvey.pdf)

[Jacobian ik math 2.0](https://roboticsnakamura.wordpress.com/wp-content/uploads/2020/06/advanced-robotics-y.-nakamura.pdf)

[Unity transform](https://docs.unity3d.com/6000.3/Documentation/ScriptReference/Transform.html)

[Unity ik](https://docs.unity3d.com/es/current/Manual/InverseKinematics.html)

[Jacobian study](https://mtsu.pressbooks.pub/robotics/chapter/chapter-4/)

[Video Jacobian explanation](https://www.youtube.com/watch?v=2_cdDGwnl80)

[Waht is DLS](https://www.educative.io/answers/what-is-depth-limited-search)

[Inverse Kinematics](https://motion.cs.illinois.edu/RoboticSystems/InverseKinematics.html)

[Coordinate descent](https://bookdown.org/rdpeng/advstatcomp/coordinate-descent.html)

[CCD 2D](https://www.ryanjuckett.com/cyclic-coordinate-descent-in-2d/)

[FABRIK](https://pubs.lib.uiowa.edu/dhm/article/31772/galley/140227/view/)
