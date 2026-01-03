using UnityEngine;

public class JacobianIK : MonoBehaviour
{
    [SerializeField] private Transform target;
    [SerializeField] private Transform poleTarget;
    [SerializeField] private Transform[] joints;
    [SerializeField] private Transform endEffector;
    [SerializeField] private float step = 300f;
    [SerializeField] private float threshold = 0.1f;
    [SerializeField] private int maxIterations = 10;

    [Header("Shouder limits (degrees)")]
    [SerializeField] private Vector2 shoulderPitchLimits = new Vector2(-60f, 90f);
    [SerializeField] private Vector2 shoulderYawLimits = new Vector2(-90f, 90f);

    [Header("Elbow limits (degrees)")]
    [SerializeField] private Vector2 elbowLimits = new Vector2(0f, 135f);

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

        PoleVector();
    }

    // só move num eixo
    private void JacobianOneAxis()
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

    //move em 2 eixos mas o cotovelo não gira
    private void Jacobian()
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

    // roda o cotovelo
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

    // jacobian com limites nas juntas
    private void JacobianWithConstrains()
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

    // definir os limites do ombro
    private void ShoulderConstrains(Transform shoulder, Vector3 error)
    {
        Vector3[] localAxes = { Vector3.forward, Vector3.up };

        foreach (Vector3 axis in localAxes)
        {
            Vector3 worldAxis = shoulder.TransformDirection(axis);
            Vector3 toEndeffector = endEffector.position - shoulder.position;

            Vector3 jacobian = Vector3.Cross(worldAxis, toEndeffector);
            float delta = Vector3.Dot(jacobian, error) * step * Time.deltaTime;

            if (Mathf.Abs(delta) < 0.0001f)
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

    // definir limites do cotovelo
    private void ElbowConstrains( Transform elbow, Vector3 error)
    {
        Vector3 worldAxis = elbow.TransformDirection(Vector3.right);
        Vector3 toEndEffector = endEffector.position - elbow.position;

        Vector3 jacobian = Vector3.Cross(worldAxis, toEndEffector);

        float delta = Vector3.Dot(jacobian, error) * step * Time.deltaTime;

        if (Mathf.Abs(delta) < 0.0001f)
            return;

        Quaternion InitialRotation = elbow.localRotation;

        elbow.Rotate(Vector3.right, delta, Space.Self);

        ClampElbowRotation();

        if (Quaternion.Angle(InitialRotation, elbow.localRotation) < 0.01f)
        {
            elbow.localRotation = InitialRotation;
        }
    }

    // limitar a rotação do ombro
    private void ClampShoulderRotation()
    {
        Transform shoulder = joints[0];

        // Converter rotação local
        Quaternion relative = Quaternion.Inverse(shoulderInitialAngle) * shoulder.localRotation;

        Vector3 euler = relative.eulerAngles;

        // [0, 360] -> [-180, 180]
        euler.x = NormalizeAngle(euler.x);
        euler.y = NormalizeAngle(euler.y);
        euler.z = NormalizeAngle(euler.z);

        // Limitar eixos
        euler.x = Mathf.Clamp(euler.x, shoulderPitchLimits.x, shoulderPitchLimits.y);
        euler.y = Mathf.Clamp(euler.y, shoulderYawLimits.x, shoulderYawLimits.y);
        euler.z = 0f;

        shoulder.localRotation = shoulderInitialAngle * Quaternion.Euler(euler);
    }

    // limitar a rotação do cotovelo
    private void ClampElbowRotation()
    {
        Transform elbow = joints[1];

        Quaternion relative = Quaternion.Inverse(elbowInitialAngle) * elbow.localRotation;

        Vector3 euler = relative.eulerAngles;

        euler.x = NormalizeAngle(euler.x);

        euler.x = Mathf.Clamp(euler.x, elbowLimits.x, elbowLimits.y);

        elbow.localRotation = elbowInitialAngle * Quaternion.Euler(euler);
    }

    // Limitar os eixos do cotovelo
    private void PoleVector()
    {
        if (poleTarget == null || joints.Length < 2)
            return;

        Transform shoulder = joints[0];
        Transform elbow = joints[1];
        Transform hand = endEffector;

        // Vetores do braço
        Vector3 shoulderToHand = hand.position - shoulder.position;
        Vector3 shoulderToElbow = elbow.position - shoulder.position;
        Vector3 shoulderToPole = poleTarget.position - shoulder.position;

        // Verifica se o braço está esticado
        float armStraight = Vector3.Dot(shoulderToElbow.normalized, (hand.position - elbow.position).normalized);

        if (armStraight > 0.98f)
            return;

        // Planos das normais
        Vector3 currentNormal = Vector3.Cross(shoulderToElbow, shoulderToHand).normalized;
        Vector3 desiredNormal = Vector3.Cross(shoulderToPole, shoulderToHand).normalized;

        if (currentNormal.sqrMagnitude < 0.0001f || desiredNormal.sqrMagnitude < 0.0001f)
            return;

        Vector3 rotationAxis = shoulderToHand.normalized;

        float angle = Vector3.SignedAngle(currentNormal, desiredNormal, rotationAxis);

        shoulder.Rotate(rotationAxis, angle * 0.2f, Space.World);
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
                Gizmos.DrawLine(joint.position, joint.position + jacobian * 0.3f);
            }

            if (poleTarget != null && joints.Length >= 2)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawCube(poleTarget.position, new Vector3 (0.05f, 0.05f, 0.05f));
                Gizmos.DrawLine(joints[1].position, poleTarget.position);
            }
        }
    }
}

