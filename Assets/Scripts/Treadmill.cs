//Treadmill.cs
/* 
GameObject which handless all of the ground elements which pass under the players.

*/

using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public class Treadmill : MonoBehaviour
{
	List<GameObject> groundSegments;
	List<GroundSegment> groundSegmentScripts;

	int currentSegmentIndex = 0;

	int segmentCounter = 0;

	float currentSegmentProgress = 0.0f;

	public Material debugMat1;
	public Material debugMat2;
	bool lastSegWasMat1 = false;


	void Awake()
	{
		groundSegments = new List<GameObject>();
		groundSegmentScripts = new List<GroundSegment>();
		//initializeGroundSegments();

		addNewGroundSegment();
		addNewGroundSegment();
		addNewGroundSegment();
	}

	void Start()
	{
		alignSegments(1);
	}

	void Update()
	{

	}

	private GameObject addNewGroundSegment()
	{
		// Instantiate a new Ground Segment GameObject and call any necessary starting methods
		GameObject gs = Instantiate(Resources.Load("Ground Segment"), transform.position, transform.rotation) as GameObject;
		gs.name = "GroundSegment" + segmentCounter;
		gs.transform.parent = transform;
		GroundSegment gsScript = gs.GetComponent<GroundSegment>();

		//move this ground segment to the end of the line
		if (groundSegments.Count > 0)
		{
			Vector3 endOfLine = groundSegments.Last().transform.localPosition;
			endOfLine.z += groundSegmentScripts.Last().getLength();
			gs.transform.localPosition = endOfLine;
		}
		

		// apply material
		if (lastSegWasMat1)
		{
			gsScript.setMaterial(debugMat2);
			lastSegWasMat1 = false;
		}
		else
		{
			gsScript.setMaterial(debugMat1);
			lastSegWasMat1 = true;
		}

		// add ground segment to data structures
		groundSegments.Add(gs);
		groundSegmentScripts.Add(gsScript);


		segmentCounter++;
		
		// gs.GetComponent<GroundSegment>().initialize(path);

		return gs;

	}

	private void alignSegments(int curSegIndex)
	{
		GameObject curSeg = groundSegments[curSegIndex];
		GameObject nextSeg;
		GameObject prevSeg;

		if (curSegIndex > 0)
		{
			nextSeg = groundSegments[curSegIndex - 1];
		}

		if (curSegIndex + 1 < groundSegments.Count)
		{
			prevSeg = groundSegments[curSegIndex + 1];
		}
	}
}