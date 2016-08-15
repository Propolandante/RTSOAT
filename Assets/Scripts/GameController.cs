// GameController.cs

using UnityEngine;

public class GameController : MonoBehaviour
{
	GameObject treadmill;
	Camera mainCamera;

	void Awake()
	{
		treadmill = GameObject.Find("Treadmill");
		mainCamera = Camera.main;
	}

	void Update()
	{
		
	}
}