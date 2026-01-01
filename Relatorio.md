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

---

## **Técnicas usadas**

Jacobian ik

---

## **Técnicas implementadas**

Jacobian ik

---

## **Problemas/erros que tive durante o trabalho**

Antes de começar o projeto, não me organizei bem com o tempo e não fiz uma boa gestão entre djd2 e cg. O que me dificultou bem mais o trabalho, por isso investi mais na investigação.

Quando comecei, tive alguns problemas na atribuição de juntas pois estava a mete-las pela ordem errada e isso fazia com que o código nao funcionasse.
Depois o meu target não parava no sitio (erro comum, tinha o target dentro do end effector, ou seja quando o end effector mexia o target também logo nunca chegava ao destino marcado).

Depois não estava a conseguir fazer com que o cotovelo rodasse se quer.

Tive alguns problemas na implementação das constrains nas juntas.

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
