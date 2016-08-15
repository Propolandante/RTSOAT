//GroundSegment.cs

using UnityEngine;

//[ExecuteInEditMode]
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class GroundSegment : MonoBehaviour
{
	public Mesh mesh;
	private float width = 16f;
	private float length = 32f;
	private int resolution = 32;

	private MeshRenderer meshRenderer;


	private void Awake()
	{
		initializeMesh();

		meshRenderer = GetComponent<MeshRenderer>();
	}

	private void initializeMesh()
	{
		mesh = generateMesh();
		GetComponent<MeshFilter>().mesh = mesh;
		//printMeshInfo();
	}

	private Mesh generateMesh()
	{
		// The ground mesh is a grid, whose heightmap is determined by a combination of 
		// authored paths and Perlin noise. 

		// First, calculate the dimensions of each grid rectangle
		int numRectsX = resolution;
		float rectX = width / resolution;
		int numRectsY = Mathf.FloorToInt(length / rectX);
		float rectY = length / numRectsY;

		Vector3[] verts = new Vector3[(numRectsX + 1) * (numRectsY + 1)];
		Vector2[] uv = new Vector2[(numRectsX + 1) * (numRectsY + 1)];

		int index = 0;
		for (int j = 0; j < numRectsY + 1; j++)
		{
			for (int i = 0; i < numRectsX + 1; i++)
			{
				float vx = i * rectX;
				float vy = 0;
				float vz = j * rectY;
				verts[index] = new Vector3(vx, vy, vz);

				float uvx = (float) vx / (float) width;
				float uvy = (float) vz / (float) length;
				uv[index] = new Vector2(uvx, uvy);

				//Debug.Log(uvx + ", " + uvy);


				index++;
			}
		}

		int[] tris = new int[numRectsX * numRectsY * 6];
		for (int rectNum = 0; rectNum < (numRectsX * numRectsY); rectNum++)
		{
			tris[rectNum * 6 + 0] = rectNum + (rectNum / numRectsX);
			tris[rectNum * 6 + 1] = rectNum + numRectsX + 1 + (rectNum / numRectsX);
			tris[rectNum * 6 + 2] = rectNum + numRectsX + 2 + (rectNum / numRectsX);
			tris[rectNum * 6 + 3] = rectNum + (rectNum / numRectsX);
			tris[rectNum * 6 + 4] = rectNum + numRectsX + 2 + (rectNum / numRectsX);
			tris[rectNum * 6 + 5] = rectNum + 1 + (rectNum / numRectsX);
		}

		Mesh generatedMesh = new Mesh();
		generatedMesh.vertices = verts;
		generatedMesh.triangles = tris;
		generatedMesh.uv = uv;

		return generatedMesh;
	}

	////////// GETTERS AND SETTERS //////////

	public float getWidth()
	{
		return width;
	}

	public float getLength()
	{
		return length;
	}

	public Material getMaterial()
	{
		return meshRenderer.materials[0];
	}

	public void setMaterial(Material m)
	{
		meshRenderer.material = m;
	}

	////////// DEBUG METHODS //////////

	private void printMeshInfo()
	{
		Debug.Log("Vertices:\n");
		foreach (Vector3 vertex in mesh.vertices)
		{
			Debug.Log("\t" + vertex + "\n");
		}

		Debug.Log("Triangles:\n");
		int countToSix = 0;
		string triString = "";
		foreach (int triVert in mesh.triangles)
		{
			countToSix++;

			if (countToSix != 1)
			{
				triString += ", ";
			}

			triString += triVert;


			if (countToSix == 6)
			{
				triString += "\n";
				Debug.Log(triString);
				triString = "";
				countToSix = 0;
			}
		}
	}
}