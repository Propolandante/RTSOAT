using UnityEngine;
using System.Collections.Generic;
using System.Linq;

[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(MeshCollider))]
public class NavMeshUtility : MonoBehaviour
{
	public Mesh mesh;

	[SerializeField]
	public List<NavMeshNode> navMeshNodes;

	GameObject clickStart;
	int startTri;
	GameObject clickDest;
	int destTri;

	void Awake()
	{
		mesh = GetComponent<MeshFilter>().mesh;
		navMeshNodes = new List<NavMeshNode>();
	}


	void Start()
	{
		//printVerts();
		//printTris();
		initializeNavMeshNodes();
	}

	void Update()
	{
		// left click to set clickStart
		if (Input.GetMouseButtonDown(0))
		{
			RaycastHit hit;
			Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
			if (Physics.Raycast(ray, out hit, 100f, 1 << 8))
			{
				if (hit.collider.gameObject == gameObject)
				{
					Debug.Log("HIT!");
					if (clickStart != null) {Destroy(clickStart);}
					clickStart = new GameObject();
					clickStart.transform.parent = gameObject.transform;
					clickStart.transform.localPosition = transform.InverseTransformPoint(hit.point);
					startTri = hit.triangleIndex;

					if (clickDest != null)
					{
						findPath(clickStart.transform.localPosition, clickDest.transform.localPosition, startTri, destTri);
					}
				}
			}
		}

		// right click to set clickDest
		if (Input.GetMouseButtonDown(1))
		{
			RaycastHit hit;
			Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
			if (Physics.Raycast(ray, out hit, 100f, 1 << 8))
			{
				if (hit.collider.gameObject == gameObject)
				{
					Debug.Log("HIT!");
					if (clickDest != null) {Destroy(clickDest);}
					clickDest = new GameObject();
					clickDest.transform.parent = gameObject.transform;
					clickDest.transform.localPosition = transform.InverseTransformPoint(hit.point);
					destTri = hit.triangleIndex;

					if (clickStart != null)
					{
						findPath(clickStart.transform.localPosition, clickDest.transform.localPosition, startTri, destTri);
					}
				}
			}
		}
	}

	private void initializeNavMeshNodes()
	{
		// Create NavMeshNodes from vertices
		for (int triNum = 0; triNum * 3 < mesh.triangles.Length; triNum++)
		{
			int a = mesh.triangles[triNum * 3];
			int b = mesh.triangles[triNum * 3 + 1];
			int c = mesh.triangles[triNum * 3 + 2];

			NavMeshNode node = new NavMeshNode(triNum, mesh.vertices[a], mesh.vertices[b], mesh.vertices[c]);
			//node.debugDrawPos = true;
			navMeshNodes.Add(node);
		}

		// Add edge information to each node
		// For each triangle...
		for (int n = 0; n < navMeshNodes.Count; n++)
		{
			// loop through every triangle we haven't yet added all neighbors for
			for (int p = n+1; p < navMeshNodes.Count; p++)
			{
				// if we've already added the max number of edges, move on
				if(navMeshNodes[n].edges.Count == 3)
				{
					break;
				}
				if (navMeshNodes[n].addEdgeIfExists(navMeshNodes[p]))
				{
					//Debug.Log(navMeshNodes[n].nodeNum + " <---> " + navMeshNodes[p].nodeNum);
				}
			}
		}
	}

	void OnDrawGizmos()
	{
		foreach (NavMeshNode node in navMeshNodes)
		{
			if (node.debugDrawPos)
			{
				Gizmos.color = Color.green;
				Gizmos.DrawSphere(transform.TransformPoint(node.position), 0.015f);
			}
			if (node.debugDrawVerts)
			{
				Gizmos.color = Color.yellow;
				foreach (Vector3 vert in node.verts)
				{
					Gizmos.DrawSphere(transform.TransformPoint(vert), 0.015f);
				}
				
			}
			// TODO debugDrawNormal

			foreach (NavMeshEdge edge in node.edges)
			{
				if (edge.debugDrawEdge)
				{
					Gizmos.color = Color.green;
					Gizmos.DrawLine(transform.TransformPoint(edge.from.position), transform.TransformPoint(edge.to.position));
				}
				if (edge.debugDrawVerts)
				{
					Gizmos.color = new Color(1f, 0.5f, 0.32f); // orange
					Gizmos.DrawSphere(transform.TransformPoint(edge.left), 0.015f);

					Gizmos.color = Color.cyan;
					Gizmos.DrawSphere(transform.TransformPoint(edge.right), 0.015f);
				}
			}
		}

		if (clickStart != null)
		{
			Gizmos.color = Color.magenta;
			Gizmos.DrawSphere(clickStart.transform.position, 0.05f);
		}

		if (clickDest != null)
		{
			Gizmos.color = Color.black;
			Gizmos.DrawSphere(clickDest.transform.position, 0.05f);
		}

		
	}

	/*
	private void printVerts()
	{
		string output = gameObject.name + " mesh vertices:\n";
		for (int v = 0; v < mesh.vertices.Length; v++)
		{
			output += v.ToString("000") + "\t" + mesh.vertices[v].ToString("F2") + "\n";
		}

		Debug.Log(output);
	}

	private void printTris()
	{
		string output = gameObject.name + " mesh triangles:\n";

		for (int triNum = 0; triNum * 3 < mesh.triangles.Length; triNum++)
		{
			int a = mesh.triangles[triNum * 3];
			int b = mesh.triangles[triNum * 3 + 1];
			int c = mesh.triangles[triNum * 3 + 2];

			output += "Triangle "+triNum.ToString("000")+" ("+a.ToString()+"-"+b.ToString()+"-"+c.ToString()+"):\n";

			output += a.ToString("000") + "\t" + mesh.vertices[a].ToString("F2") + "\n";
			output += b.ToString("000") + "\t" + mesh.vertices[b].ToString("F2") + "\n";
			output += c.ToString("000") + "\t" + mesh.vertices[c].ToString("F2") + "\n";
		}

		Debug.Log(output);
	}
	*/

	private bool pointIsInNodesPlane(Vector3 p, NavMeshNode n)
	{
		// scalar equation of a plane: n.x(p.x - a.x) + n.y(p.y - a.y) + n.z(p.z - a.z) = 0
		// n is the normal, p is the point we are checking, a is any point on the plane (such as point A of the triangle)

		Vector3 a = n.verts[0];

		float x_component = n.normal.x * (p.x - a.x);
		float y_component = n.normal.y * (p.y - a.y);
		float z_component = n.normal.z * (p.z - a.z);

		return Mathf.Approximately(x_component+y_component+z_component, 0);
	}

	private NavMeshNode nodeSharesPlaneWithPoint(Vector3 p)
	{
		foreach (NavMeshNode node in navMeshNodes)
		{
			if (pointIsInNodesPlane(p, node))
			{
				Debug.Log("point shares plane with node " + node.nodeNum);
				node.debugDrawVerts = true;
				node.debugDrawNormal = true;
				return node;
			}
		}

		return null;
	}

	
	public List<Vector3> findPath(Vector3 start, Vector3 dest, int st, int dt)
	{

		float[] costs = new float[navMeshNodes.Count];
		float[] fScores = new float[navMeshNodes.Count];
		int[] prevNodes = new int[navMeshNodes.Count];
		List<int> openSet = new List<int>();
		List<int> closedSet = new List<int>();
		NavMeshNode startNode;
		NavMeshNode destNode;
		
		Debug.Assert(navMeshNodes[st].nodeNum == st, "start tri mismatch: " + navMeshNodes[st].nodeNum + " != " + st);
		Debug.Assert(navMeshNodes[dt].nodeNum == dt, "dest tri mismatch: " + navMeshNodes[dt].nodeNum + " != " + dt);

		// determine which node the Start position is in
		startNode = navMeshNodes[st];
		//startNode.debugDrawVerts = true;
		//startNode.debugDrawNormal = true;

		// determine which node the Destination position is in
		destNode = navMeshNodes[dt];
		//destNode.debugDrawVerts = true;
		//destNode.debugDrawNormal = true;

		// initialize the Cost and Heuristic values for all nodes
		for (int i = 0; i < costs.Length; i++) {costs[i] = Mathf.Infinity;}
		for (int i = 0; i < fScores.Length; i++) {fScores[i] = Mathf.Infinity;}
		for (int i = 0; i < prevNodes.Length; i++) {prevNodes[i] = -1;}
		// measure the Start node's cost and heuristic and add it to the open
		costs[startNode.nodeNum] = 0;
		fScores[startNode.nodeNum] = heuristic(0, startNode.position);
		openSet.Add(startNode.nodeNum);

		// while we still have available nodes to explore 
		while (openSet.Count > 0)
		{
			// choose the one with the best heuristic (closest to the goal)
			float bestOpenNodeValue = Mathf.Infinity;
			int currentNodeNum = openSet[0];
			foreach (int n in openSet)
			{
				if (fScores[n] < bestOpenNodeValue)
				{
					currentNodeNum = n;
					bestOpenNodeValue = fScores[n];
				}
			}

			// if the node we chose IS the node that houses the destination point, we're done searching!
			if (navMeshNodes[currentNodeNum] == destNode)
			{
				constructPath(prevNodes, currentNodeNum, start, dest);
			}

			// explore the neighbor nodes
			openSet.Remove(currentNodeNum);
			closedSet.Add(currentNodeNum);
			foreach (NavMeshEdge e in navMeshNodes[currentNodeNum].edges)
			{
				if (closedSet.Contains(e.to.nodeNum))
				{
					continue;
				}
				float tempCost = costs[currentNodeNum] + Vector3.Distance(navMeshNodes[currentNodeNum].position, e.to.position);
				if (!openSet.Contains(e.to.nodeNum))
				{
					openSet.Add(e.to.nodeNum);
				}
				else if (tempCost >= costs[e.to.nodeNum])
				{
					continue;
				}

				prevNodes[e.to.nodeNum] = currentNodeNum;
				costs[e.to.nodeNum] = tempCost;
				fScores[e.to.nodeNum] = costs[e.to.nodeNum] + heuristic(0, e.to.position);
			}


		}

		// If we reach here, then there IS no path.
		// TODO We should probably do something besides return an empty list.
		return new List<Vector3>();
	}

	private float heuristic(int type, Vector3 pos)
	{
		// No heuristic: A* will function identically to Dijkstra's Algorithm
		if (type == 0)
		{
			return 0;
		}
		else
		{
			Debug.LogAssertion("heuristic type not recognized: " + type, gameObject);
			return 0;
		}
	}

	private List<Vector3> constructPath(int[] prevNodes, int endNodeNum, Vector3 startPoint, Vector3 endPoint)
	{
		Debug.Log("constructPath()");
		List<NavMeshNode> nodePath = new List<NavMeshNode>();

		// turn the int array into a series of Nodes
		int n = endNodeNum;
		while (n != -1)
		{
			navMeshNodes[n].debugDrawPos = true;
			nodePath.Add(navMeshNodes[n]);
			n = prevNodes[n];
		}
		nodePath.Reverse();

		// if startPoint and endPoint are in the same node, then nodePath is empty.

		return simpleStupidFunnel(nodePath, startPoint, endPoint, true);

	}

	private List<Vector3> simpleStupidFunnel(List<NavMeshNode> nodes, Vector3 startPoint, Vector3 endPoint, bool debug = false)
	{
		Debug.Log("simpleStupidFunnel()");
		// Assumptions: startPoint is inside of nodes[0], and endPoint is inside of nodes.Last()
		int edgeCount = nodes.Count - 1;
		Vector3[] leftBounds = new Vector3[edgeCount];
		Vector3[] rightBounds = new Vector3[edgeCount];
		List<NavMeshEdge> edgePath = new List<NavMeshEdge>();
		Vector3 fLeft;
		Vector3 fRight;

		// initialize pathfinding data
		for (int n = 0; n < nodes.Count - 1; n++)
		{
			// determine the NavMeshEdge connecting this node to the next
			bool edgeFound = false;
			foreach (NavMeshEdge e in nodes[n].edges)
			{
				if (e.to == nodes[n + 1])
				{
					edgeFound = true;
					// add the left and right points to the corresponding bounds arrays
					leftBounds[n] = e.left;
					rightBounds[n] = e.right;
					edgePath.Add(e);
					if (debug) {e.debugDrawVerts = true;}
					break;
				}
			}

			if (edgeFound)
			{
				//Debug.Log("Edge Found");
			}
			else
			{
				Debug.LogAssertion("No edge found between nodes " + nodes[n].nodeNum + " and " + nodes[n+1].nodeNum);
			}
		}

		// debug draw boundaries
		if (debug)
		{
			float normalOffset = 0f;
			Color funnelColor = Color.black;
			float funnelSeconds = 300;
			Debug.Assert(leftBounds.Length == rightBounds.Length, "Different number of left and right bounds: " + leftBounds.Length + " " + rightBounds.Length);
			Debug.DrawLine(startPoint + (nodes[0].normal * normalOffset), leftBounds[0] + (nodes[0].normal * normalOffset), funnelColor, funnelSeconds);
			Debug.DrawLine(startPoint + (nodes[0].normal * normalOffset), rightBounds[0] + (nodes[0].normal * normalOffset), funnelColor, funnelSeconds);
			for (int b = 0; b < leftBounds.Length - 1; b++)
			{
				if (leftBounds[b] != leftBounds[b+1])
				{
					Debug.DrawLine(leftBounds[b] + (nodes[b].normal * normalOffset), leftBounds[b+1] + (nodes[b+1].normal * normalOffset), funnelColor, funnelSeconds);
				}
				if (rightBounds[b] != rightBounds[b+1])
				{
					Debug.DrawLine(rightBounds[b] + (nodes[b].normal * normalOffset), rightBounds[b+1] + (nodes[b+1].normal * normalOffset), funnelColor, funnelSeconds);
				}
			}
			Debug.DrawLine(leftBounds[leftBounds.Length-1] + (nodes[leftBounds.Length-1].normal * normalOffset), endPoint + (nodes[leftBounds.Length].normal * normalOffset), funnelColor, funnelSeconds);
			Debug.DrawLine(rightBounds[leftBounds.Length-1] + (nodes[rightBounds.Length-1].normal * normalOffset), endPoint + (nodes[rightBounds.Length].normal * normalOffset), funnelColor, funnelSeconds);
		}

		// Simple Stupid Funnel Algorithm
		// http://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html
		fLeft = leftBounds[0];
		fRight = rightBounds[0];
		List<Vector3> waypoints = new List<Vector3>();
		Vector3 waypointNormal = nodes[0].normal;
		waypoints.Add(startPoint);
		int currentEdge = 1;
		while (currentEdge < edgeCount)
		{
			// TODO check for a change in normals across an edge. There's probably something special to do in that case.
			// first check left point

			// if ithe edge's left point is not the same exact position as the funnel's left point
			if (fLeft != edgePath[currentEdge].left)
			{
				// if the edge's left point is to the right of the funnel's left side
				if (!isLeft(waypoints.Last(), fLeft, edgePath[currentEdge].left, waypointNormal))
				{
					//if the point is to the left of the funnel's right edge
					if (isLeft(waypoints.Last(), fRight, edgePath[currentEdge].left, waypointNormal))
					{
						// then this edge's left point is inside the funnel, and we can narrow the funnel
						fLeft = edgePath[currentEdge].left;
					}
					// otherwise we've hit a corner, need to drop a new waypoint.
					else
					{
						// TODO figure out if we can be certain that the waypoint's position is on the current edge.
						// if it is, then it can be considered "past" that edge, and we can increment the current edge TWICE:
						// Once, to designate the funnel's edges at the next edge's left and right (just like Edge 0)
						// And again, to begin the next loop evaluating the edge after that! Just be sure to check if we've reached the end!
						waypoints.Add(fRight);
						waypointNormal = edgePath[currentEdge].from.normal;
						fLeft = edgePath[currentEdge].left;
						fRight = edgePath[currentEdge].right;
						currentEdge++;
						break;
					}
				}
				// otherwise, this edge's left point is outside the funnel
				else
				{
					// do nothing
				}
			}
			// if the edge's left point IS the exact same position as the funnel's left point
			else
			{
				// do nothing
			}

			// if the edge's right point is not the same exact position as the funnel's right point
			if (fRight != edgePath[currentEdge].right)
			{
				// if the edge's right point is to the left of the funnel's right side
				if (isLeft(waypoints.Last(), fRight, edgePath[currentEdge].right, waypointNormal))
				{
					// if the edge's right point is to the right of the funnel's left edge
					if (isLeft(waypoints.Last(), fLeft, edgePath[currentEdge].right,waypointNormal))
					{
						// then this edge's right point is inside the funnel, and we can narrow the funnel
						fRight = edgePath[currentEdge].right;
					}
					// otherwise we've hit a corner, and need to drop a new waypoint.
					else
					{
						waypoints.Add(fLeft);
						waypointNormal = edgePath[currentEdge].from.normal; // TODO fix this line. This is not true if we've looped a few times before dropping a waypoint.
						fLeft = edgePath[currentEdge].left;
						fRight = edgePath[currentEdge].right;
						currentEdge++;
						break;
					}
				}
			}
			// if the edge's right point IS the exact same position as the funnel's right point
			else
			{
				// do nothing
			}


			currentEdge++;
		}

		// once we're reached the edge before the destination, we do a special check.
		// if the endPoint is outside the funnel, then we drop a waypoint at the funnel's edge.
		// also, iif the endPoint is around a corner, drop a waypoint at the corner

		// I think this might be wrong. I think I'm supposed to process the final edge normally and just check the endPoint like an edge bound. 

		if (isLeft(waypoints.Last(), fLeft, endPoint, waypointNormal))
		{
			waypoints.Add(fLeft);

			if (isLeft(waypoints.Last(), edgePath.Last().left, endPoint, waypointNormal))
			{
				waypoints.Add(edgePath.Last().left);
			}
		}
		else if (!isLeft(waypoints.Last(), fRight, endPoint, waypointNormal))
		{
			waypoints.Add(fRight);

			if (!isLeft(waypoints.Last(), edgePath.Last().right, endPoint, waypointNormal))
			{
				waypoints.Add(edgePath.Last().right);
			}
		}

		waypoints.Add(endPoint);

		//Debug.Log(nodes.Count);

		return waypoints;
	}

	private bool isLeft(Vector3 funnelStart, Vector3 funnelEnd, Vector3 pointToCheck, Vector3 up)
	{
		// TODO handle case where funnelStart and funnelEnd are the same. Pretty sure it's just {return true;}
		Vector3 lineFunnel = funnelEnd - funnelStart;
		Vector3 linePoint = pointToCheck - funnelStart;
		Vector3 cross = Vector3.Cross(linePoint, lineFunnel).normalized;
		if (cross.normalized == up.normalized) {return true;}
		else {return false;}
	}

	

	[System.Serializable]
	public class NavMeshNode
	{
		public int nodeNum;
		public Vector3 position;
		public List<NavMeshEdge> edges;
		public List<Vector3> verts;
		public Vector3 normal;

		public bool debugDrawPos = false;
		public bool debugDrawVerts = false;
		public bool debugDrawNormal = false;


		public NavMeshNode()
		{
			nodeNum = -1;
			position = Vector3.zero;
			edges = new List<NavMeshEdge>();
			verts = new List<Vector3>();
			normal = Vector3.zero;
		}

		public NavMeshNode(int n, Vector3 center)
		{
			nodeNum = n;
			position = center;
			edges = new List<NavMeshEdge>();
			verts = new List<Vector3>();
			normal = Vector3.zero;
		}

		public NavMeshNode(int n, Vector3 a, Vector3 b, Vector3 c)
		{
			nodeNum = n;
			float cx = (a.x + b.x + c.x) / 3f;
			float cy = (a.y + b.y + c.y) / 3f;
			float cz = (a.z + b.z + c.z) / 3f;
			position = new Vector3(cx, cy, cz);
			edges = new List<NavMeshEdge>();
			verts = new List<Vector3>();
			verts.Add(a);
			verts.Add(b);
			verts.Add(c);
			normal = calcNormal();
		}

		public bool pointInTriangle(Vector3 p)
		{
			// determine if point p is in the triangle

			return true;
		}

		public bool addEdgeIfExists(NavMeshNode potentialNeighbor)
		{
			List<Vector3> sharedVerts = getSharedVerts(potentialNeighbor);
			Debug.Assert(sharedVerts.Count <= 3, "More than 3 shared verts on a triangle? " + sharedVerts.Count);
			Debug.Assert(sharedVerts.Count != 3, "Checked to see if a triangle was its own neighbor.");
			if (sharedVerts.Count == 2)
			{
				NavMeshEdge edgeTo;
				NavMeshEdge edgeFrom;
				// calculate edge cost
				float cost = Vector3.Distance(this.position, potentialNeighbor.position);
				// determine left vert for edge FROM this node TO the neighbor
				Vector3 edgeRay = potentialNeighbor.position - this.position;
				Vector3 vertRay = sharedVerts[0] - this.position;
				if (Vector3.Cross(edgeRay, vertRay).normalized == normal)
				{
					edgeTo = new NavMeshEdge(this, potentialNeighbor, sharedVerts[1], sharedVerts[0], cost);
					edgeFrom = new NavMeshEdge(potentialNeighbor, this, sharedVerts[0], sharedVerts[1], cost);
				}
				else
				{
					edgeTo = new NavMeshEdge(this, potentialNeighbor, sharedVerts[0], sharedVerts[1], cost);
					edgeFrom = new NavMeshEdge(potentialNeighbor, this, sharedVerts[1], sharedVerts[0], cost);
				}

				edges.Add(edgeTo);
				potentialNeighbor.edges.Add(edgeFrom);
				return true;
			}
			else
			{
				return false;
			}

		}

		public List<Vector3> getSharedVerts(NavMeshNode potentialNeighbor)
		{
			List<Vector3> sharedVerts = new List<Vector3>();

			for (int v = 0; v < verts.Count; v++)
			{
				for (int w = 0; w < potentialNeighbor.verts.Count; w++)
				{
					if (verts[v]==potentialNeighbor.verts[w])
					{
						sharedVerts.Add(verts[v]);
						break;
					}
				}
			}
			
			return sharedVerts;
		}

		private Vector3 calcNormal()
		{
			// TODO fix normal calculation
			Debug.Assert(verts.Count >= 3, "Can't calculate normal without 3 verts");
			Vector3 ab = verts[1] - verts[0];
			Vector3 ac = verts[2] - verts[0];
			return Vector3.Cross(ab, ac).normalized;
		}
	}

	public class NavMeshEdge
	{
		public NavMeshNode from;
		public NavMeshNode to;
		public Vector3 left;
		public Vector3 right;
		public Vector3 center;
		public float cost;

		public bool debugDrawEdge = false;
		public bool debugDrawVerts = false;

		public NavMeshEdge(NavMeshNode f, NavMeshNode t, Vector3 l, Vector3 r, float c)
		{
			from = f;
			to = t;
			left = l;
			right = r;
			center = new Vector3((l.x + r.x)/2f, (l.y + r.y)/2f, (l.z + r.z)/2f);
			cost = c;

		}
	}

	
}

