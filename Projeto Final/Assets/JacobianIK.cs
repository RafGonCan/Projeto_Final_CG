using UnityEngine;

public class JacobianIK : MonoBehaviour
{
    [SerializeField] private Transform target;
    [SerializeField] private Transform[] joints;
    [SerializeField] private Transform endEffector;
    [SerializeField] private float step = 5.0f;
    [SerializeField] private float threshold = 0.1f;


    // Update is called once per frame
    void Update()
    {
        JacobianWithElbowRotation();
    }

    private void JacobianOneAxis() // só move num eixo
    {
        // saber os limites das juntas
        Vector3 error = target.position - endEffector.position;

        if (error.magnitude < threshold)
            return;

        // calcular a matriz jacobiana
        for (int i = joints.Length - 1; i >= 0; i--)
        {
            Transform joint = joints[i];
            // eixo da junta y
            Vector3 jointAxis = joint.right;

            // vetor da junta até ao end efector
            Vector3 toEndEffector = endEffector.position - joint.position;

            // coluna jacobiana
            Vector3 jacobianCollumn = Vector3.Cross(jointAxis, toEndEffector);

            // atualizar a posição da junta
            float deltaAngle = Vector3.Dot(jacobianCollumn, error) * step;

            //ignora se a rotação for muito pequena
            if (Mathf.Abs(deltaAngle) < 0.01f)
                continue;

            // aplicar a rotação à junta no mundo
            joint.Rotate(jointAxis, deltaAngle, Space.World);
        }
    }

    private void Jacobian() //move em 2 eixos mas o cotovelo não gira
    {
        Vector3 error = target.position - endEffector.position;

        if (error.magnitude < threshold)
            return;

        for (int i = joints.Length - 1; i >= 0; i--)
        {
            Transform joint = joints[i];

            Vector3 jointAxis;

            if (i == 0)
            {
                jointAxis = joint.up;
            }

            else
            {
                jointAxis = joint.forward;
            }


            Vector3 toEndEffector = endEffector.position - joint.position;
            Vector3 jacobianCollumn = Vector3.Cross(jointAxis, toEndEffector);

            float deltaAngle = Vector3.Dot(jacobianCollumn, error) * step;

            if (Mathf.Abs(deltaAngle) < 0.01f)
                continue;

            joint.Rotate(jointAxis, deltaAngle, Space.World);
        }
    }

    private void JacobianWithElbowRotation()
    {
        Vector3 error = target.position - endEffector.position;

        if (error.magnitude < threshold)
            return;

        for (int i = joints.Length - 1; i >= 0; i--)
        {
            Transform joint = joints[i];
            Vector3[] rotationAxes;

            if (i == 0) // Ombro
            {
                rotationAxes = new Vector3[] {Vector3.forward, Vector3.up};
            }
            else if (i == 1) // Cotovelo
            {
                rotationAxes = new Vector3[] {Vector3.right};
            }
            else
            {
                rotationAxes = new Vector3[] {Vector3.right};
            }

            foreach (Vector3 axis in rotationAxes)
            {
                // Converter valores locais para globais
                Vector3 worldAxis = joint.TransformDirection(axis);
                
                Vector3 toEndEffector = endEffector.position - joint.position;
                Vector3 jacobianCollumn = Vector3.Cross(worldAxis, toEndEffector);

                float deltaAngle = Vector3.Dot(jacobianCollumn, error) * step;

                if (Mathf.Abs(deltaAngle) < 0.01f)
                    continue;

                joint.Rotate(worldAxis, deltaAngle, Space.World);
            }
        }
    }
}

