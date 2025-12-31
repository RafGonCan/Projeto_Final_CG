# ***Inverse Kinematics***

## **Pesquisa**

### **O que é inverse kinematics?**

Inverse kinematics é o estudo do movimento de um objeto sem ter em conta o torque e forças aplicadas ao mesmo.
Ou seja, é um conjunto de calculos das variáveis das juntas e dos conectores a um ponto final no objeto (PFO). Dando a posição e a orientação do PFO conseguimos calcular as todas as variaveis consoante as juntas e os conectores incluindo posição, rotação e orientação.

![ ](Imagens/InverseKinematics_1.jpg)

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

![ ](Imagens/Jacobian.jpg)

---

## **Técnicas usadas**

Matriz jacobiana

---

## **Técnicas implementadas**

Matriz jacobiana sem restrições

---

## **Problemas/erros que tive durante o trabalho**

Antes de começar o projeto, não me organizei bem com o tempo e não fiz uma boa gestão entre djd2 e cg. O que me dificultou bem mais o trabalho, por isso investi mais na investigação.

Quando comecei, tive alguns problemas na atribuição de juntas pois estava a mete-las pela ordem errada e isso fazia com que o código nao funcionasse.
Depois o meu target não parava no sitio (erro comum, tinha o target dentro do end effector, ou seja quando o end effector mexia o target também logo nunca chegava ao destino marcado).

Depois não estava a conseguir fazer com que o cotovelo rodasse se quer.

---

## **Bibliografia**

[Forward kinematics vs inverse kinematics](https://irisdynamics.com/articles/forward-and-inverse-kinematics)

[What is inverse kinematics](https://www.mathworks.com/discovery/inverse-kinematics.html)

[Jacobian ik](https://medium.com/unity3danimation/overview-of-jacobian-ik-a33939639ab2)

[Jacobian ik 2.0](https://medium.com/unity3danimation/analytical-jacobian-ik-cb3df86edf00)

[Jacobian ik math](chrome-extension://efaidnbmnnnibpcajpcglclefindmkaj/https://www.cs.cmu.edu/~15464-s13/lectures/lecture6/iksurvey.pdf)

[Jacobian ik math 2.0](chrome-extension://efaidnbmnnnibpcajpcglclefindmkaj/https://roboticsnakamura.wordpress.com/wp-content/uploads/2020/06/advanced-robotics-y.-nakamura.pdf)

[Unity transform](https://docs.unity3d.com/6000.3/Documentation/ScriptReference/Transform.html)

[Unity ik](https://docs.unity3d.com/es/current/Manual/InverseKinematics.html)

[Jacobian study](https://mtsu.pressbooks.pub/robotics/chapter/chapter-4/)
