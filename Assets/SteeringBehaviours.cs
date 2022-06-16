using UnityEngine;

public static class SteeringBehaviours2D
{
    public delegate Vector2 BehaviourDelegate();

    public static Vector2 Seek(Vector2 _Position, Vector2 _Velocity, float _MaxVelocity, Vector2 _TPosition)
    {
        Vector2 DesiredVelocity = (_TPosition - _Position).normalized * _MaxVelocity;
        return DesiredVelocity - _Velocity;
    }

    public static Vector2 Flee(Vector2 _Position, Vector2 _Velocity, float _MaxVelocity, Vector2 _TPosition)
    {
        Vector2 DesiredVelocity = (_Position - _TPosition).normalized * _MaxVelocity;
        return DesiredVelocity - _Velocity;
    }

    public static Vector2 Pursue(Vector2 _Position, Vector2 _Velocity, float _MaxVelocity, float _MaxPrediction, Vector2 _TPosition, Vector2 _TVelocity)
    {
        float Speed = _Velocity.magnitude;
        float Distance = Vector2.Distance(_Position, _TPosition);
        float Prediction = (Speed <= Distance / _MaxPrediction) ? _MaxPrediction : Distance / Speed;

        Vector2 SeekTarget = _TPosition + (_TVelocity * Prediction);

        return Seek(_Position, _Velocity, _MaxVelocity, SeekTarget);
    }

    public static Vector2 Evade(Vector2 _Position, Vector2 _Velocity, float _MaxVelocity, float _MaxPrediction, Vector2 _TPosition, Vector2 _TVelocity)
    {
        float Speed = _Velocity.magnitude;
        float Distance = Vector2.Distance(_Position, _TPosition);
        float Prediction = (Speed <= Distance / _MaxPrediction) ? _MaxPrediction : Distance / Speed;

        Vector2 SeekTarget = _TPosition + (_TVelocity * Prediction);

        return Flee(_Position, _Velocity, _MaxVelocity, SeekTarget);
    }

    public static Vector2 Arrive(Vector2 _Position, Vector2 _Velocity, float _MaxVelocity, float _SlowingRadius, Vector2 _TPosition)
    {
        Vector2 TargetOffset = _TPosition - _Position;
        float Distance = TargetOffset.magnitude;

        if (Distance < _SlowingRadius)
        {
            Vector2 v2fDesiredVelocity = TargetOffset.normalized * _MaxVelocity * (Distance / _SlowingRadius);
            return v2fDesiredVelocity - _Velocity;
        }

        return Seek(_Position, _Velocity, _MaxVelocity, _TPosition);
    }

    public static Vector2 Wander(Vector2 _Position, Vector2 _Velocity, float _MaxVelocity, float _WanderRadius, float _WanderDistance, ref float _WanderTimer, float _WanderCooldown, ref Vector2 _TRelative, float _Deltatime = 1)
    {
        if (_WanderTimer <= 0)
        {
            float WanderAngle = Random.Range(0.0f, 360.0f) * Mathf.Deg2Rad;
            _TRelative = _WanderRadius * new Vector2(Mathf.Cos(WanderAngle), Mathf.Sin(WanderAngle));
            _TRelative += _Velocity.normalized * _WanderDistance;

            _WanderTimer = _WanderCooldown;
        }
        else
        {
            _WanderTimer -= _Deltatime;
        }

        Vector2 TPosition = _TRelative + _Position;

        return Seek(_Position, _Velocity, _MaxVelocity, TPosition);
    }

    public static Vector2 Sepparation(Vector2 _Position, Vector2 _Velocity, float _MaxVelocity, Vector2[] _TPositions)
    {
        if (_TPositions.Length <= 0) return Vector2.zero;

        Vector2 DesiredVelocity = Vector2.zero;

        foreach (Vector2 TPosition in _TPositions)
        {
            Vector2 Difference = _Position - TPosition;
            if (Difference == Vector2.zero) continue;

            DesiredVelocity += Difference.normalized / Difference.magnitude;
        }
        DesiredVelocity /= _TPositions.Length;
        DesiredVelocity = DesiredVelocity.normalized * _MaxVelocity;

        return DesiredVelocity - _Velocity;
    }

    public static Vector2 ObstacleAvoidance(Vector2 _Position, Vector2 _TransformUp, Vector2 _TransformRight, Vector2 _Velocity, float _MaxVelocity, float _LocalSpaceWidth, float _TRadiusMultiply, Collider2D[] _TColliders, float fDefaultRadius = 1)
    {
        Vector3 PriorityTPosition = Vector3.zero;
        float PriorityTRadius = 0;
        float PriorityTDistance = Mathf.Infinity;

        foreach (Collider2D _TCollider in _TColliders)
        {
            Vector2 TPosition = _TCollider.transform.position;

            //Ignore any Obstacles behind the agent
            if (Vector3.Dot((TPosition - _Position).normalized, _TransformUp) <= 0) continue;

            //Expand Obstacle Radius
            CircleCollider2D TCircleCollider = _TCollider.GetComponent<CircleCollider2D>();
            float TRadius = (TCircleCollider == null) ? fDefaultRadius : TCircleCollider.radius;
            TRadius += _LocalSpaceWidth / 2.0f;
            TRadius *= _TRadiusMultiply;

            //Find intersections
            Vector2 Intersect1, Intersect2;
            CircleLineSegmentIntesect(_Position, _Position + _Velocity, TPosition, TRadius, out Intersect1, out Intersect2);

            //Compare intersections
            if (Vector2.Distance(_Position, Intersect1) < PriorityTDistance)
            {
                PriorityTPosition = Intersect1;
                PriorityTRadius = TRadius;
                PriorityTDistance = Vector2.Distance(_Position, Intersect1);
            }

            if (Vector2.Distance(_Position, Intersect2) < PriorityTDistance)
            {
                PriorityTPosition = Intersect2;
                PriorityTRadius = TRadius;
                PriorityTDistance = Vector2.Distance(_Position, Intersect2);
            }
        }

        //Checks whether any obstacles have been detected
        if (PriorityTDistance == Mathf.Infinity) return Vector2.zero;

        //Add the force to avoid the priority obstacle
        return (_TransformUp * -PriorityTPosition.magnitude) + (_TransformRight * (PriorityTRadius - PriorityTPosition.y) * PriorityTPosition.magnitude);
    }

    public static Vector2 WallAvoidance(Vector2 _Position, Vector2 _Velocity, Vector2 _TransformUp, float _MaxVelocity, int _WallLayerMask, float _WallSightLength, uint _RayNumber)
    {
        Vector2 TPosition = Vector2.zero;

        for (uint i = 0; i < _RayNumber; i++)
        {
            RaycastHit2D Hit = Physics2D.Raycast
            (
                _Position,
                Quaternion.Euler(0.0f, 0.0f, i * 360.0f / _RayNumber) * _TransformUp,
                _WallSightLength,
                _WallLayerMask
            );

            if (Hit.collider == null) continue;
            if (TPosition != Vector2.zero && Vector2.Distance(Hit.point, TPosition) >= Vector2.Distance(_Position, TPosition)) continue;

            TPosition = Hit.point;
        }

        if (TPosition == Vector2.zero) return Vector2.zero;

        return Flee(_Position, _Velocity, _MaxVelocity, TPosition);
    }

    public static Vector2 WeightedTruncatedRunningSumWithPrioritization(BehaviourDelegate[] SteeringBehaviours, float _MaxForce)
    {
        Vector2 Force = Vector2.zero;
        foreach (BehaviourDelegate SteeringBehaviour in SteeringBehaviours)
        {
            Vector2 CalculatedForce = SteeringBehaviour();
            if ((Force + CalculatedForce).magnitude < _MaxForce)
            {
                Force += CalculatedForce;
            }
            else
            {
                Vector2 Intersect1, Intersect2;
                CircleLineSegmentIntesect(Force, Force + CalculatedForce, Vector2.zero, _MaxForce, out Intersect1, out Intersect2);
                if (Intersect1 != Intersect2) break;

                Force = Intersect1;
                break;
            }
        }

        return Force;
    }

    //https://answers.unity.com/questions/1658184/circle-line-intersection-points.html
    static void CircleLineSegmentIntesect(Vector2 p1, Vector2 p2, Vector2 center, float radius, out Vector2 i1, out Vector2 i2)
    {
        //  get the distance between X and Z on the segment
        Vector2 dp = p2 - p1;

        float a = Vector2.Dot(dp, dp);
        float b = 2 * Vector2.Dot(dp, p1 - center);
        float c = Vector2.Dot(center, center) - 2 * Vector2.Dot(center, p1) + Vector2.Dot(p1, p1) - radius * radius;
        float bb4ac = b * b - 4 * a * c;
        if (Mathf.Abs(a) < float.Epsilon || bb4ac < 0)
        {
            i1 = p2;
            i2 = p2;
            return;
        }
        float mu1 = (-b + Mathf.Sqrt(bb4ac)) / (2 * a);
        float mu2 = (-b - Mathf.Sqrt(bb4ac)) / (2 * a);
        Vector2[] sect = new Vector2[2];
        i1 = p1 + mu1 * (p2 - p1);
        i2 = p1 + mu2 * (p2 - p1);
    }
}