using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding;
using Pathfinding.RVO;

/** AI for following paths.
 * This AI is the default movement script which comes with the A* Pathfinding Project.
 * It is in no way required by the rest of the system, so feel free to write your own. But I hope this script will make it easier
 * to set up movement for the characters in your game. This script is not written for high performance, so I do not recommend using it for large groups of units.
 * \n
 * \n
 * This script will try to follow a target transform, in regular intervals, the path to that target will be recalculated.
 * It will on FixedUpdate try to move towards the next point in the path.
 * However it will only move in the forward direction, but it will rotate around it's Y-axis
 * to make it reach the target.
 *
 * \section variables Quick overview of the variables
 * In the inspector in Unity, you will see a bunch of variables. You can view detailed information further down, but here's a quick overview.\n
 * The #repathRate determines how often it will search for new paths, if you have fast moving targets, you might want to set it to a lower value.\n
 * The #target variable is where the AI will try to move, it can be a point on the ground where the player has clicked in an RTS for example.
 * Or it can be the player object in a zombie game.\n
 * The speed is self-explanatory, so is turningSpeed, however #slowdownDistance might require some explanation.
 * It is the approximate distance from the target where the AI will start to slow down. Note that this doesn't only affect the end point of the path
 * but also any intermediate points, so be sure to set #forwardLook and #pickNextWaypointDist to a higher value than this.\n
 * #pickNextWaypointDist is simply determines within what range it will switch to target the next waypoint in the path.\n
 * #forwardLook will try to calculate an interpolated target point on the current segment in the path so that it has a distance of #forwardLook from the AI\n
 * Below is an image illustrating several variables as well as some internal ones, but which are relevant for understanding how it works.
 * Note that the #forwardLook range will not match up exactly with the target point practically, even though that's the goal.
 * \shadowimage{aipath_variables.png}
 * This script has many movement fallbacks.
 * If it finds a NavmeshController, it will use that, otherwise it will look for a character controller, then for a rigidbody and if it hasn't been able to find any
 * it will use Transform.Translate which is guaranteed to always work.
 */
[RequireComponent(typeof(Seeker))]
[AddComponentMenu("Pathfinding/AI/AIPath (3D)")]
[HelpURL("http://arongranberg.com/astar/docs/class_a_i_path.php")]
public class AIPath : MonoBehaviour {
	/** Determines how often it will search for new paths.
	 * If you have fast moving targets or AIs, you might want to set it to a lower value.
	 * The value is in seconds between path requests.
	 */
	public float repathRate = 0.5F;

	/** Target to move towards.
	 * The AI will try to follow/move towards this target.
	 * It can be a point on the ground where the player has clicked in an RTS for example, or it can be the player object in a zombie game.
	 */
	public Transform target;

	/** Enables or disables searching for paths.
	 * Setting this to false does not stop any active path requests from being calculated or stop it from continuing to follow the current path.
	 * \see #canMove
	 */
	public bool canSearch = true;

	/** Enables or disables movement.
	 * \see #canSearch */
	public bool canMove = true;

	/** Maximum velocity.
	 * This is the maximum speed in world units per second.
	 */
	public float speed = 3;

	/** Rotation speed.
	 * Rotation is calculated using Quaternion.SLerp. This variable represents the damping, the higher, the faster it will be able to rotate.
	 */
	public float turningSpeed = 5;

	/** Distance from the target point where the AI will start to slow down.
	 * Note that this doesn't only affect the end point of the path
	 * but also any intermediate points, so be sure to set #forwardLook and #pickNextWaypointDist to a higher value than this
	 */
	public float slowdownDistance = 0.6F;

	/** Determines within what range it will switch to target the next waypoint in the path */
	public float pickNextWaypointDist = 2;

	/** Target point is Interpolated on the current segment in the path so that it has a distance of #forwardLook from the AI.
	 * See the detailed description of AIPath for an illustrative image */
	public float forwardLook = 1;

	/** Distance to the end point to consider the end of path to be reached.
	 * When this has been reached, the AI will not move anymore until the target changes and OnTargetReached will be called.
	 */
	public float endReachedDistance = 0.2F;

	/** Do a closest point on path check when receiving path callback.
	 * Usually the AI has moved a bit between requesting the path, and getting it back, and there is usually a small gap between the AI
	 * and the closest node.
	 * If this option is enabled, it will simulate, when the path callback is received, movement between the closest node and the current
	 * AI position. This helps to reduce the moments when the AI just get a new path back, and thinks it ought to move backwards to the start of the new path
	 * even though it really should just proceed forward.
	 */
	public bool closestOnPathCheck = true;

	protected float minMoveScale = 0.05F;

	/** Cached Seeker component */
	protected Seeker seeker;

	/** Cached Transform component */
	protected Transform tr;

	/** Time when the last path request was sent */
	protected float lastRepath = -9999;

	/** Current path which is followed */
	protected Path path;

	/** Cached CharacterController component */
	protected CharacterController controller;


	/** Cached Rigidbody component */
	protected Rigidbody rigid;

	/** Current index in the path which is current target */
	protected int currentWaypointIndex = 0;

	/** Holds if the end-of-path is reached
	 * \see TargetReached */
	protected bool targetReached = false;

	/** Only when the previous path has been returned should be search for a new path */
	protected bool canSearchAgain = true;

	protected Vector3 lastFoundWaypointPosition;
	protected float lastFoundWaypointTime = -9999;

	public bool TargetReached {
		get {
			return targetReached;
		}
	}

	private bool startHasRun = false;

	protected virtual void Awake () {
		seeker = GetComponent<Seeker>();

		tr = transform;

		controller = GetComponent<CharacterController>();
		rigid = GetComponent<Rigidbody>();
	}
	protected virtual void Start () {
		startHasRun = true;
		OnEnable();
	}

	protected virtual void OnEnable () {
		lastRepath = -9999;
		canSearchAgain = true;

		lastFoundWaypointPosition = GetFeetPosition();

		if (startHasRun) {
			seeker.pathCallback += OnPathComplete;

			StartCoroutine(RepeatTrySearchPath());
		}
	}

	public void OnDisable () {
		if (seeker != null && !seeker.IsDone()) seeker.GetCurrentPath().Error();

		if (path != null) path.Release(this);
		path = null;

		seeker.pathCallback -= OnPathComplete;
	}
	protected IEnumerator RepeatTrySearchPath () {
		while (true) {
			float v = TrySearchPath();
			yield return new WaitForSeconds(v);
		}
	}
	public float TrySearchPath () {
		if (Time.time - lastRepath >= repathRate && canSearchAgain && canSearch && target != null) {
			SearchPath();
			return repathRate;
		} else {
			float v = repathRate - (Time.time-lastRepath);
			return v < 0 ? 0 : v;
		}
	}

	public virtual void SearchPath () {
		if (target == null) throw new System.InvalidOperationException("Target is null");

		lastRepath = Time.time;
		Vector3 targetPosition = target.position;

		canSearchAgain = false;

		seeker.StartPath(GetFeetPosition(), targetPosition);
	}

	public virtual void OnTargetReached () {
		
	}

	public virtual void OnPathComplete (Path _p) {
		ABPath p = _p as ABPath;

		if (p == null) throw new System.Exception("This function only handles ABPaths, do not use special path types");

		canSearchAgain = true;

		p.Claim(this);

		if (p.error) {
			p.Release(this);
			return;
		}

		if (path != null) path.Release(this);

		path = p;

		currentWaypointIndex = 0;
		targetReached = false;

		if (closestOnPathCheck) {
			
			Vector3 p1 = Time.time - lastFoundWaypointTime < 0.3f ? lastFoundWaypointPosition : p.originalStartPoint;
			Vector3 p2 = GetFeetPosition();
			Vector3 dir = p2-p1;
			float magn = dir.magnitude;
			dir /= magn;
			int steps = (int)(magn/pickNextWaypointDist);


			for (int i = 0; i <= steps; i++) {
				CalculateVelocity(p1);
				p1 += dir;
			}
		}
	}

	public virtual Vector3 GetFeetPosition () {
		if (controller != null) {
			return tr.position - Vector3.up*controller.height*0.5F;
		}

		return tr.position;
	}

	public virtual void Update () {
		if (!canMove) { return; }

		Vector3 dir = CalculateVelocity(GetFeetPosition());

		RotateTowards(targetDirection);

		if (controller != null) {
			controller.SimpleMove(dir);
		} else if (rigid != null) {
			rigid.AddForce(dir);
		} else {
			tr.Translate(dir*Time.deltaTime, Space.World);
		}
	}


	protected Vector3 targetPoint;

	protected Vector3 targetDirection;

	protected float XZSqrMagnitude (Vector3 a, Vector3 b) {
		float dx = b.x-a.x;
		float dz = b.z-a.z;

		return dx*dx + dz*dz;
	}
	protected Vector3 CalculateVelocity (Vector3 currentPosition) {
		if (path == null || path.vectorPath == null || path.vectorPath.Count == 0) return Vector3.zero;

		List<Vector3> vPath = path.vectorPath;

		if (vPath.Count == 1) {
			vPath.Insert(0, currentPosition);
		}

		if (currentWaypointIndex >= vPath.Count) { currentWaypointIndex = vPath.Count-1; }

		if (currentWaypointIndex <= 1) currentWaypointIndex = 1;

		while (true) {
			if (currentWaypointIndex < vPath.Count-1) {
				float dist = XZSqrMagnitude(vPath[currentWaypointIndex], currentPosition);
				if (dist < pickNextWaypointDist*pickNextWaypointDist) {
					lastFoundWaypointPosition = currentPosition;
					lastFoundWaypointTime = Time.time;
					currentWaypointIndex++;
				} else {
					break;
				}
			} else {
				break;
			}
		}

		Vector3 dir = vPath[currentWaypointIndex] - vPath[currentWaypointIndex-1];
		Vector3 targetPosition = CalculateTargetPoint(currentPosition, vPath[currentWaypointIndex-1], vPath[currentWaypointIndex]);


		dir = targetPosition-currentPosition;
		dir.y = 0;
		float targetDist = dir.magnitude;

		float slowdown = Mathf.Clamp01(targetDist / slowdownDistance);

		this.targetDirection = dir;
		this.targetPoint = targetPosition;

		if (currentWaypointIndex == vPath.Count-1 && targetDist <= endReachedDistance) {
			if (!targetReached) { targetReached = true; OnTargetReached(); }

			return Vector3.zero;
		}

		Vector3 forward = tr.forward;
		float dot = Vector3.Dot(dir.normalized, forward);
		float sp = speed * Mathf.Max(dot, minMoveScale) * slowdown;


		if (Time.deltaTime > 0) {
			sp = Mathf.Clamp(sp, 0, targetDist/(Time.deltaTime*2));
		}
		return forward*sp;
	}

	/** Rotates in the specified direction.
	 * Rotates around the Y-axis.
	 * \see turningSpeed
	 */
	protected virtual void RotateTowards (Vector3 dir) {
		if (dir == Vector3.zero) return;

		Quaternion rot = tr.rotation;
		Quaternion toTarget = Quaternion.LookRotation(dir);

		rot = Quaternion.Slerp(rot, toTarget, turningSpeed*Time.deltaTime);
		Vector3 euler = rot.eulerAngles;
		euler.z = 0;
		euler.x = 0;
		rot = Quaternion.Euler(euler);

		tr.rotation = rot;
	}

	protected Vector3 CalculateTargetPoint (Vector3 p, Vector3 a, Vector3 b) {
		a.y = p.y;
		b.y = p.y;

		float magn = (a-b).magnitude;
		if (magn == 0) return a;

		float closest = Mathf.Clamp01(VectorMath.ClosestPointOnLineFactor(a, b, p));
		Vector3 point = (b-a)*closest + a;
		float distance = (point-p).magnitude;

		float lookAhead = Mathf.Clamp(forwardLook - distance, 0.0F, forwardLook);

		float offset = lookAhead / magn;
		offset = Mathf.Clamp(offset+closest, 0.0F, 1.0F);
		return (b-a)*offset + a;
		Debug.Log (a + b);
	}
}
