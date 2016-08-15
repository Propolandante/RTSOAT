using UnityEngine;

public class CharacterMotor : MonoBehaviour
{
	[SerializeField] float movementSpeed = 10f;
	GameObject target;

	Rigidbody rigidbody;
	bool isGrounded;
	Vector3 groundNormal;


	void Start()
	{
		if (target != null)
		{
			target = new GameObject();
			target.transform.parent = gameObject.transform;
			target.transform.localPosition = Vector3.zero;
		}
	}

	void Update()
	{
		if (target != null)
		{
			seekTarget();
		}
	}

	void seekTarget()
	{

	}

	void CheckGroundStatus()
	{
		RaycastHit hitInfo;
		#if UNITY_EDITOR
		// helper to visualise the ground check ray in the scene view
		//Debug.DrawLine(transform.position + (Vector3.up * 0.1f), transform.position + (Vector3.up * 0.1f) + (Vector3.down * m_GroundCheckDistance));
		#endif

		if (Physics.Raycast(transform.position, Vector3.down, out hitInfo, 0.01f))
		{
			groundNormal = hitInfo.normal;
			isGrounded = true;
		}
		else
		{
			isGrounded = false;
			groundNormal = Vector3.up;
		}
	}
}