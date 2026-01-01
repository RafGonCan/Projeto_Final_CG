using UnityEngine;

public class JacobianIK : MonoBehaviour
{
    [SerializeField] private Transform target;
    [SerializeField] private Transform[] joints;
    [SerializeField] private Transform endEffector;
    [SerializeField] private float step = 60.0f;
    [SerializeField] private float threshold = 0.1f;
    [SerializeField] private int maxIterations = 10;

    [Header("Shouder limits (degrees)")]
    [SerializeField] private Vector2 shoulderPitchLimits = new Vector2(-60f, 90f);
    [SerializeField] private Vector2 shoulderYawLimits = new Vector2(-90f, 90f);

    [Header("Elbow limits (degrees)")]
    [SerializeField] private Vector2 elbowLimits = new Vector2(0f, 135.0f);

    private Quaternion shoulderInitialAngle;
    private Quaternion elbowInitialAngle;

    private void Start()
    {
        if (joints != null && joints.Length > 0)
            shoulderInitialAngle = joints[0].localRotation;

        if (joints.Length > 1)
            elbowInitialAngle = joints[1].localRotation;
    }

    // Update is called once per frame
    void Update()
    {
        for (int i = 0; i < maxIterations; i++)
        {
            JacobianWithConstrains();
        }
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

    private void JacobianWithElbowRotation() // roda o cotovelo
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
                rotationAxes = new Vector3[] { Vector3.up, Vector3.forward };
            }
            else // Cotovelo
            {
                rotationAxes = new Vector3[] { Vector3.right };
            }

            foreach (Vector3 axis in rotationAxes)
            {
                // Converter valores locais para globais
                Vector3 worldAxis = joint.TransformDirection(axis);

                Vector3 toEndEffector = endEffector.position - joint.position;
                Vector3 jacobianCollumn = Vector3.Cross(worldAxis, toEndEffector);

                float deltaAngle = Vector3.Dot(jacobianCollumn, error) * step * Time.deltaTime;

                if (Mathf.Abs(deltaAngle) < 0.01f)
                    continue;

                joint.Rotate(axis, deltaAngle, Space.Self);
            }

            if (i == 0)
                ClampShoulderRotation();
        }
    }

    private void JacobianWithConstrains() // jacobian com limites nas juntas
    {
        Vector3 error = target.position - endEffector.position;

        if (error.magnitude < threshold)
            return;

        for (int i = joints.Length - 1; i >= 0; i--)
        {
            if (i == 0)
                ShoulderConstrains(joints[i], error);

            else
              ElbowConstrains(joints[i], error);
        }
    }

    private void ShoulderConstrains(Transform shoulder, Vector3 error) // definir os limites do ombro
    {
        Vector3[] localAxes = { Vector3.forward, Vector3.up };

        foreach (Vector3 axis in localAxes)
        {
            Vector3 worldAxis = shoulder.TransformDirection(axis);
            Vector3 toEndeffector = endEffector.position - shoulder.position;

            Vector3 jacobian = Vector3.Cross(worldAxis, toEndeffector);
            float delta = Vector3.Dot(jacobian, error) * step;

            if (Mathf.Abs(delta) < 0.01f)
                continue;

            Quaternion InitialRotation = shoulder.localRotation;

            shoulder.Rotate(axis, delta, Space.Self);

            ClampShoulderRotation();

            if (Quaternion.Angle(InitialRotation, shoulder.localRotation) < 0.01f)
            {
                shoulder.localRotation = InitialRotation;
            }
        }
    }

    private void ElbowConstrains( Transform elbow, Vector3 error) // definir limites do cotovelo
    {
        Vector3 worldAxis = elbow.TransformDirection(Vector3.right);
        Vector3 toEndEffector = endEffector.position - elbow.position;

        Vector3 jacobian = Vector3.Cross(worldAxis, toEndEffector);

        float delta = Vector3.Dot(jacobian, error) * step;

        if (Mathf.Abs(delta) < 0.01f)
            return;

        elbow.Rotate(worldAxis, delta, Space.World);

        Quaternion relative = Quaternion.Inverse(elbowInitialAngle) * elbow.localRotation;
        
        Vector3 euler = relative.eulerAngles;

        euler.x = NormalizeAngle(euler.x);
        euler.x = Mathf.Clamp(euler.x, elbowLimits.x, elbowLimits.y);

        elbow.localRotation = elbowInitialAngle * Quaternion.Euler(euler);
    }

    private void ClampShoulderRotation() // limitar a rotação do ombro
    {
        Transform shoulder = joints[0];

        // Converter rotação local
        Quaternion relative = Quaternion.Inverse(shoulderInitialAngle) * shoulder.localRotation;

        Vector3 euler = relative.eulerAngles;

        // [0, 360] -> [-180, 180]
        euler.x = NormalizeAngle(euler.x);
        euler.y = NormalizeAngle(euler.y);

        // Limitar eixos
        euler.x = Mathf.Clamp(euler.x, shoulderPitchLimits.x, shoulderPitchLimits.y);
        euler.y = Mathf.Clamp(euler.y, shoulderYawLimits.x, shoulderYawLimits.y);

        shoulder.localRotation = shoulderInitialAngle * Quaternion.Euler(euler);
    }

    float NormalizeAngle(float angle)
    {
        if (angle > 180f)
        {
            angle -= 360f;
        }
        
        return angle;
    }

    void OnDrawGizmos()
    {
        if (joints == null || joints.Length == 0 || endEffector == null || target == null)
            return;

        // Desenhar alvo
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(target.position, 0.05f);

        // Vector erro
        Gizmos.color = Color.green;
        Gizmos.DrawLine(endEffector.position, target.position);

        // Eixos jacobianos
        for (int i = 0; i < joints.Length; i++)
        {
            Transform joint = joints[i];

            Vector3[] rotationAxes;

            if (i == 0) // Ombro
            {
                rotationAxes = new Vector3[] { Vector3.forward, Vector3.up };
            }
            else // Cotovelo
            {
                rotationAxes = new Vector3[] { Vector3.right };
            }

            foreach (Vector3 axis in rotationAxes)
            {
                Vector3 worldAxis = joint.TransformDirection(axis);
                Vector3 toEnd = endEffector.position - joint.position;

                Vector3 jacobian = Vector3.Cross(worldAxis, toEnd).normalized;

                Gizmos.color = Color.blue;
                Gizmos.DrawLine (joint.position, joint.position + jacobian * 0.3f);
            }
        }
    }
}

