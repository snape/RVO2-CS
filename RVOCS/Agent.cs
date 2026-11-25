/*
 * Agent.cs
 * RVO2 Library C#
 *
 * SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

using System;
using System.Collections.Generic;

namespace RVO
{
    /// <summary>Defines an agent in the simulation.</summary>
    internal class Agent
    {
        internal IList<KeyValuePair<float, Agent>> _agentNeighbors = new List<KeyValuePair<float, Agent>>();
        internal IList<KeyValuePair<float, Obstacle>> _obstacleNeighbors = new List<KeyValuePair<float, Obstacle>>();
        internal IList<Line> _orcaLines = new List<Line>();
        internal Vector2 _position;
        internal Vector2 _prefVelocity;
        internal Vector2 _velocity;
        internal int _id = 0;
        internal int _maxNeighbors = 0;
        internal float _maxSpeed = 0.0f;
        internal float _neighborDist = 0.0f;
        internal float _radius = 0.0f;
        internal float _timeHorizon = 0.0f;
        internal float _timeHorizonObst = 0.0f;

        private Vector2 _newVelocity;

        /// <summary>Computes the neighbors of this agent.</summary>
        internal void ComputeNeighbors()
        {
            _obstacleNeighbors.Clear();
            float range = _timeHorizonObst * _maxSpeed + _radius;
            float rangeSq = range * range;
            Simulator.Instance._kdTree.ComputeObstacleNeighbors(this, rangeSq);

            _agentNeighbors.Clear();

            if (_maxNeighbors > 0)
            {
                rangeSq = _neighborDist * _neighborDist;
                Simulator.Instance._kdTree.ComputeAgentNeighbors(this, ref rangeSq);
            }
        }

        /// <summary>Computes the new velocity of this agent.</summary>
        internal void ComputeNewVelocity()
        {
            _orcaLines.Clear();

            float invTimeHorizonObst = 1.0f / _timeHorizonObst;

            /* Create obstacle ORCA lines. */
            for (int i = 0; i < _obstacleNeighbors.Count; ++i)
            {

                Obstacle obstacle1 = _obstacleNeighbors[i].Value;
                Obstacle obstacle2 = obstacle1._next;

                Vector2 relativePosition1 = obstacle1._point - _position;
                Vector2 relativePosition2 = obstacle2._point - _position;

                /*
                 * Check if velocity obstacle of obstacle is already taken care
                 * of by previously constructed obstacle ORCA lines.
                 */
                bool alreadyCovered = false;

                for (int j = 0; j < _orcaLines.Count; ++j)
                {
                    if (RVOMath.Det(invTimeHorizonObst * relativePosition1 - _orcaLines[j].Point, _orcaLines[j].Direction) - invTimeHorizonObst * _radius >= -RVOMath.RVO_EPSILON && RVOMath.Det(invTimeHorizonObst * relativePosition2 - _orcaLines[j].Point, _orcaLines[j].Direction) - invTimeHorizonObst * _radius >= -RVOMath.RVO_EPSILON)
                    {
                        alreadyCovered = true;

                        break;
                    }
                }

                if (alreadyCovered)
                {
                    continue;
                }

                /* Not yet covered. Check for collisions. */
                float distSq1 = RVOMath.AbsSq(relativePosition1);
                float distSq2 = RVOMath.AbsSq(relativePosition2);

                float radiusSq = _radius * _radius;

                Vector2 obstacleVector = obstacle2._point - obstacle1._point;
                float s = (-relativePosition1 * obstacleVector) / RVOMath.AbsSq(obstacleVector);
                float distSqLine = RVOMath.AbsSq(-relativePosition1 - s * obstacleVector);

                Line line;

                if (s < 0.0f && distSq1 <= radiusSq)
                {
                    /* Collision with left vertex. Ignore if non-convex. */
                    if (obstacle1._convex)
                    {
                        line.Point = new Vector2(0.0f, 0.0f);
                        line.Direction = RVOMath.Normalize(new Vector2(-relativePosition1.Y, relativePosition1.X));
                        _orcaLines.Add(line);
                    }

                    continue;
                }
                else if (s > 1.0f && distSq2 <= radiusSq)
                {
                    /*
                     * Collision with right vertex. Ignore if non-convex or if
                     * it will be taken care of by neighboring obstacle.
                     */
                    if (obstacle2._convex && RVOMath.Det(relativePosition2, obstacle2._direction) >= 0.0f)
                    {
                        line.Point = new Vector2(0.0f, 0.0f);
                        line.Direction = RVOMath.Normalize(new Vector2(-relativePosition2.Y, relativePosition2.X));
                        _orcaLines.Add(line);
                    }

                    continue;
                }
                else if (s >= 0.0f && s <= 1.0f && distSqLine <= radiusSq)
                {
                    /* Collision with obstacle segment. */
                    line.Point = new Vector2(0.0f, 0.0f);
                    line.Direction = -obstacle1._direction;
                    _orcaLines.Add(line);

                    continue;
                }

                /*
                 * No collision. Compute legs. When obliquely viewed, both legs
                 * can come from a single vertex. Legs extend cut-off line when
                 * non-convex vertex.
                 */

                Vector2 leftLegDirection, rightLegDirection;

                if (s < 0.0f && distSqLine <= radiusSq)
                {
                    /*
                     * Obstacle viewed obliquely so that left vertex
                     * defines velocity obstacle.
                     */
                    if (!obstacle1._convex)
                    {
                        /* Ignore obstacle. */
                        continue;
                    }

                    obstacle2 = obstacle1;

                    float leg1 = MathF.Sqrt(distSq1 - radiusSq);
                    leftLegDirection = new Vector2(relativePosition1.X * leg1 - relativePosition1.Y * _radius, relativePosition1.X * _radius + relativePosition1.Y * leg1) / distSq1;
                    rightLegDirection = new Vector2(relativePosition1.X * leg1 + relativePosition1.Y * _radius, -relativePosition1.X * _radius + relativePosition1.Y * leg1) / distSq1;
                }
                else if (s > 1.0f && distSqLine <= radiusSq)
                {
                    /*
                     * Obstacle viewed obliquely so that
                     * right vertex defines velocity obstacle.
                     */
                    if (!obstacle2._convex)
                    {
                        /* Ignore obstacle. */
                        continue;
                    }

                    obstacle1 = obstacle2;

                    float leg2 = MathF.Sqrt(distSq2 - radiusSq);
                    leftLegDirection = new Vector2(relativePosition2.X * leg2 - relativePosition2.Y * _radius, relativePosition2.X * _radius + relativePosition2.Y * leg2) / distSq2;
                    rightLegDirection = new Vector2(relativePosition2.X * leg2 + relativePosition2.Y * _radius, -relativePosition2.X * _radius + relativePosition2.Y * leg2) / distSq2;
                }
                else
                {
                    /* Usual situation. */
                    if (obstacle1._convex)
                    {
                        float leg1 = MathF.Sqrt(distSq1 - radiusSq);
                        leftLegDirection = new Vector2(relativePosition1.X * leg1 - relativePosition1.Y * _radius, relativePosition1.X * _radius + relativePosition1.Y * leg1) / distSq1;
                    }
                    else
                    {
                        /* Left vertex non-convex; left leg extends cut-off line. */
                        leftLegDirection = -obstacle1._direction;
                    }

                    if (obstacle2._convex)
                    {
                        float leg2 = MathF.Sqrt(distSq2 - radiusSq);
                        rightLegDirection = new Vector2(relativePosition2.X * leg2 + relativePosition2.Y * _radius, -relativePosition2.X * _radius + relativePosition2.Y * leg2) / distSq2;
                    }
                    else
                    {
                        /* Right vertex non-convex; right leg extends cut-off line. */
                        rightLegDirection = obstacle1._direction;
                    }
                }

                /*
                 * Legs can never point into neighboring edge when convex
                 * vertex, take cutoff-line of neighboring edge instead. If
                 * velocity projected on "foreign" leg, no constraint is added.
                 */

                Obstacle leftNeighbor = obstacle1._previous;

                bool isLeftLegForeign = false;
                bool isRightLegForeign = false;

                if (obstacle1._convex && RVOMath.Det(leftLegDirection, -leftNeighbor._direction) >= 0.0f)
                {
                    /* Left leg points into obstacle. */
                    leftLegDirection = -leftNeighbor._direction;
                    isLeftLegForeign = true;
                }

                if (obstacle2._convex && RVOMath.Det(rightLegDirection, obstacle2._direction) <= 0.0f)
                {
                    /* Right leg points into obstacle. */
                    rightLegDirection = obstacle2._direction;
                    isRightLegForeign = true;
                }

                /* Compute cut-off centers. */
                Vector2 leftCutOff = invTimeHorizonObst * (obstacle1._point - _position);
                Vector2 rightCutOff = invTimeHorizonObst * (obstacle2._point - _position);
                Vector2 cutOffVector = rightCutOff - leftCutOff;
                float cutOffLengthSq = RVOMath.AbsSq(cutOffVector);
                bool isPointCutOff = obstacle1 == obstacle2 || cutOffLengthSq == 0.0f;

                /* Project current velocity on velocity obstacle. */

                /* Check if current velocity is projected on cutoff circles. */
                float t = isPointCutOff ? 0.5f : ((_velocity - leftCutOff) * cutOffVector) / cutOffLengthSq;
                float tLeft = (_velocity - leftCutOff) * leftLegDirection;
                float tRight = (_velocity - rightCutOff) * rightLegDirection;

                if ((t < 0.0f && tLeft < 0.0f) || (isPointCutOff && tLeft < 0.0f && tRight < 0.0f))
                {
                    /* Project on left cut-off circle. */
                    Vector2 unitW = NormalizeCutOffDirection(_velocity - leftCutOff, obstacle1, obstacle2);

                    line.Direction = new Vector2(unitW.Y, -unitW.X);
                    line.Point = leftCutOff + _radius * invTimeHorizonObst * unitW;
                    _orcaLines.Add(line);

                    continue;
                }
                else if (t > 1.0f && tRight < 0.0f)
                {
                    /* Project on right cut-off circle. */
                    Vector2 unitW = NormalizeCutOffDirection(_velocity - rightCutOff, obstacle2, obstacle1);

                    line.Direction = new Vector2(unitW.Y, -unitW.X);
                    line.Point = rightCutOff + _radius * invTimeHorizonObst * unitW;
                    _orcaLines.Add(line);

                    continue;
                }

                /*
                 * Project on left leg, right leg, or cut-off line, whichever is
                 * closest to velocity.
                 */
                float distSqCutoff = (t < 0.0f || t > 1.0f || isPointCutOff) ? float.PositiveInfinity : RVOMath.AbsSq(_velocity - (leftCutOff + t * cutOffVector));
                float distSqLeft = tLeft < 0.0f ? float.PositiveInfinity : RVOMath.AbsSq(_velocity - (leftCutOff + tLeft * leftLegDirection));
                float distSqRight = tRight < 0.0f ? float.PositiveInfinity : RVOMath.AbsSq(_velocity - (rightCutOff + tRight * rightLegDirection));

                if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
                {
                    /* Project on cut-off line. */
                    line.Direction = -obstacle1._direction;
                    line.Point = leftCutOff + _radius * invTimeHorizonObst * new Vector2(-line.Direction.Y, line.Direction.X);
                    _orcaLines.Add(line);

                    continue;
                }

                if (distSqLeft <= distSqRight)
                {
                    /* Project on left leg. */
                    if (isLeftLegForeign)
                    {
                        continue;
                    }

                    line.Direction = leftLegDirection;
                    line.Point = leftCutOff + _radius * invTimeHorizonObst * new Vector2(-line.Direction.Y, line.Direction.X);
                    _orcaLines.Add(line);

                    continue;
                }

                /* Project on right leg. */
                if (isRightLegForeign)
                {
                    continue;
                }

                line.Direction = -rightLegDirection;
                line.Point = rightCutOff + _radius * invTimeHorizonObst * new Vector2(-line.Direction.Y, line.Direction.X);
                _orcaLines.Add(line);
            }

            int numObstLines = _orcaLines.Count;

            float invTimeHorizon = 1.0f / _timeHorizon;

            /* Create agent ORCA lines. */
            for (int i = 0; i < _agentNeighbors.Count; ++i)
            {
                Agent other = _agentNeighbors[i].Value;

                Vector2 relativePosition = other._position - _position;
                Vector2 relativeVelocity = _velocity - other._velocity;
                float distSq = RVOMath.AbsSq(relativePosition);
                float combinedRadius = _radius + other._radius;
                float combinedRadiusSq = combinedRadius * combinedRadius;

                Line line;
                Vector2 u;

                if (distSq > combinedRadiusSq)
                {
                    /* No collision. */
                    Vector2 w = relativeVelocity - invTimeHorizon * relativePosition;

                    /* Vector from cutoff center to relative velocity. */
                    float wLengthSq = RVOMath.AbsSq(w);
                    float dotProduct1 = w * relativePosition;

                    if (dotProduct1 < 0.0f && dotProduct1 * dotProduct1 > combinedRadiusSq * wLengthSq)
                    {
                        /* Project on cut-off circle. */
                        float wLength = MathF.Sqrt(wLengthSq);
                        Vector2 unitW = w / wLength;

                        line.Direction = new Vector2(unitW.Y, -unitW.X);
                        u = (combinedRadius * invTimeHorizon - wLength) * unitW;
                    }
                    else
                    {
                        /* Project on legs. */
                        float leg = MathF.Sqrt(distSq - combinedRadiusSq);

                        if (RVOMath.Det(relativePosition, w) > 0.0f)
                        {
                            /* Project on left leg. */
                            line.Direction = new Vector2(relativePosition.X * leg - relativePosition.Y * combinedRadius, relativePosition.X * combinedRadius + relativePosition.Y * leg) / distSq;
                        }
                        else
                        {
                            /* Project on right leg. */
                            line.Direction = -new Vector2(relativePosition.X * leg + relativePosition.Y * combinedRadius, -relativePosition.X * combinedRadius + relativePosition.Y * leg) / distSq;
                        }

                        float dotProduct2 = relativeVelocity * line.Direction;
                        u = dotProduct2 * line.Direction - relativeVelocity;
                    }
                }
                else
                {
                    /* Collision. Project on cut-off circle of time timeStep. */
                    float invTimeStep = 1.0f / Simulator.Instance.TimeStep;

                    /* Vector from cutoff center to relative velocity. */
                    Vector2 w = relativeVelocity - invTimeStep * relativePosition;

                    float wLength = RVOMath.Abs(w);
                    Vector2 unitW = w / wLength;

                    line.Direction = new Vector2(unitW.Y, -unitW.X);
                    u = (combinedRadius * invTimeStep - wLength) * unitW;
                }

                line.Point = _velocity + 0.5f * u;
                _orcaLines.Add(line);
            }

            int lineFail = LinearProgram2(_orcaLines, _maxSpeed, _prefVelocity, false, out _newVelocity);

            if (lineFail < _orcaLines.Count)
            {
                LinearProgram3(_orcaLines, numObstLines, lineFail, _maxSpeed, ref _newVelocity);
            }
        }

        private Vector2 NormalizeCutOffDirection(Vector2 cutOffOffset, Obstacle obstacle, Obstacle otherObstacle)
        {
            Vector2 unitW = RVOMath.Normalize(cutOffOffset);

            if (unitW.X == 0.0f && unitW.Y == 0.0f)
            {
                if (obstacle != otherObstacle)
                {
                    Vector2 segmentDir = obstacle._next == otherObstacle ? obstacle._direction : otherObstacle._direction;
                    return new Vector2(segmentDir.Y, -segmentDir.X);
                }

                Vector2 away = _position - obstacle._point;
                float dist = RVOMath.Abs(away);
                return dist > 0.0f ? away / dist : new Vector2(1.0f, 0.0f);
            }

            return unitW;
        }

        /// <summary>Inserts an agent neighbor into the set of neighbors of this
        /// agent.</summary>
        ///
        /// <param name="agent">A pointer to the agent to be inserted.</param>
        /// <param name="rangeSq">The squared range around this agent.</param>
        internal void InsertAgentNeighbor(Agent agent, ref float rangeSq)
        {
            if (this != agent)
            {
                float distSq = RVOMath.AbsSq(_position - agent._position);

                if (distSq < rangeSq)
                {
                    if (_agentNeighbors.Count < _maxNeighbors)
                    {
                        _agentNeighbors.Add(new KeyValuePair<float, Agent>(distSq, agent));
                    }

                    int i = _agentNeighbors.Count - 1;

                    while (i != 0 && distSq < _agentNeighbors[i - 1].Key)
                    {
                        _agentNeighbors[i] = _agentNeighbors[i - 1];
                        --i;
                    }

                    _agentNeighbors[i] = new KeyValuePair<float, Agent>(distSq, agent);

                    if (_agentNeighbors.Count == _maxNeighbors)
                    {
                        rangeSq = _agentNeighbors[_agentNeighbors.Count - 1].Key;
                    }
                }
            }
        }

        /// <summary>Inserts a static obstacle neighbor into the set of neighbors
        /// of this agent.</summary>
        ///
        /// <param name="obstacle">The number of the static obstacle to be
        /// inserted.</param>
        /// <param name="rangeSq">The squared range around this agent.</param>
        internal void InsertObstacleNeighbor(Obstacle obstacle, float rangeSq)
        {
            Obstacle nextObstacle = obstacle._next;

            float distSq = RVOMath.DistSqPointLineSegment(obstacle._point, nextObstacle._point, _position);

            if (distSq < rangeSq)
            {
                _obstacleNeighbors.Add(new KeyValuePair<float, Obstacle>(distSq, obstacle));

                int i = _obstacleNeighbors.Count - 1;

                while (i != 0 && distSq < _obstacleNeighbors[i - 1].Key)
                {
                    _obstacleNeighbors[i] = _obstacleNeighbors[i - 1];
                    --i;
                }
                _obstacleNeighbors[i] = new KeyValuePair<float, Obstacle>(distSq, obstacle);
            }
        }

        /// <summary>Updates the two-dimensional position and two-dimensional
        /// velocity of this agent.</summary>
        internal void Update()
        {
            _velocity = _newVelocity;

            if (RVOMath.AbsSq(_velocity) > _maxSpeed * _maxSpeed)
            {
                _velocity = RVOMath.Normalize(_velocity) * _maxSpeed;
            }

            _position += _velocity * Simulator.Instance.TimeStep;
        }

        /// <summary>Solves a one-dimensional linear program on a specified line
        /// subject to linear constraints defined by lines and a circular
        /// constraint.</summary>
        ///
        /// <returns>True if successful.</returns>
        ///
        /// <param name="lines">Lines defining the linear constraints.</param>
        /// <param name="lineNo">The specified line constraint.</param>
        /// <param name="radius">The radius of the circular constraint.</param>
        /// <param name="optVelocity">The optimization velocity.</param>
        /// <param name="directionOpt">True if the direction should be optimized.
        /// </param>
        /// <param name="result">A reference to the result of the linear program.
        /// </param>
        private bool LinearProgram1(IList<Line> lines, int lineNo, float radius, Vector2 optVelocity, bool directionOpt, ref Vector2 result)
        {
            float dotProduct = lines[lineNo].Point * lines[lineNo].Direction;
            float discriminant = dotProduct * dotProduct + radius * radius - RVOMath.AbsSq(lines[lineNo].Point);

            if (discriminant < 0.0f)
            {
                /* Max speed circle fully invalidates line lineNo. */
                return false;
            }

            float sqrtDiscriminant = MathF.Sqrt(discriminant);
            float tLeft = -dotProduct - sqrtDiscriminant;
            float tRight = -dotProduct + sqrtDiscriminant;

            for (int i = 0; i < lineNo; ++i)
            {
                float denominator = RVOMath.Det(lines[lineNo].Direction, lines[i].Direction);
                float numerator = RVOMath.Det(lines[i].Direction, lines[lineNo].Point - lines[i].Point);

                if (MathF.Abs(denominator) <= RVOMath.RVO_EPSILON)
                {
                    /* Lines lineNo and i are (almost) parallel. */
                    if (numerator < 0.0f)
                    {
                        return false;
                    }

                    continue;
                }

                float t = numerator / denominator;

                if (denominator >= 0.0f)
                {
                    /* Line i bounds line lineNo on the right. */
                    tRight = Math.Min(tRight, t);
                }
                else
                {
                    /* Line i bounds line lineNo on the left. */
                    tLeft = Math.Max(tLeft, t);
                }

                if (tLeft > tRight)
                {
                    return false;
                }
            }

            if (directionOpt)
            {
                /* Optimize direction. */
                if (optVelocity * lines[lineNo].Direction > 0.0f)
                {
                    /* Take right extreme. */
                    result = lines[lineNo].Point + tRight * lines[lineNo].Direction;
                }
                else
                {
                    /* Take left extreme. */
                    result = lines[lineNo].Point + tLeft * lines[lineNo].Direction;
                }
            }
            else
            {
                /* Optimize closest point. */
                float t = lines[lineNo].Direction * (optVelocity - lines[lineNo].Point);

                if (t < tLeft)
                {
                    result = lines[lineNo].Point + tLeft * lines[lineNo].Direction;
                }
                else if (t > tRight)
                {
                    result = lines[lineNo].Point + tRight * lines[lineNo].Direction;
                }
                else
                {
                    result = lines[lineNo].Point + t * lines[lineNo].Direction;
                }
            }

            return true;
        }

        /// <summary>Solves a two-dimensional linear program subject to linear
        /// constraints defined by lines and a circular constraint.</summary>
        ///
        /// <returns>The number of the line it fails on, and the number of lines
        /// if successful.</returns>
        ///
        /// <param name="lines">Lines defining the linear constraints.</param>
        /// <param name="radius">The radius of the circular constraint.</param>
        /// <param name="optVelocity">The optimization velocity.</param>
        /// <param name="directionOpt">True if the direction should be optimized.
        /// </param>
        /// <param name="result">A reference to the result of the linear program.
        /// </param>
        private int LinearProgram2(IList<Line> lines, float radius, Vector2 optVelocity, bool directionOpt, out Vector2 result)
        {
            if (directionOpt)
            {
                /*
                 * Optimize direction. Note that the optimization velocity is of
                 * unit length in this case.
                 */
                result = optVelocity * radius;
            }
            else if (RVOMath.AbsSq(optVelocity) > radius * radius)
            {
                /* Optimize closest point and outside circle. */
                result = RVOMath.Normalize(optVelocity) * radius;
            }
            else
            {
                /* Optimize closest point and inside circle. */
                result = optVelocity;
            }

            for (int i = 0; i < lines.Count; ++i)
            {
                if (RVOMath.Det(lines[i].Direction, lines[i].Point - result) > 0.0f)
                {
                    /* Result does not satisfy constraint i. Compute new optimal result. */
                    Vector2 tempResult = result;
                    if (!LinearProgram1(lines, i, radius, optVelocity, directionOpt, ref result))
                    {
                        result = tempResult;

                        return i;
                    }
                }
            }

            return lines.Count;
        }

        /// <summary>Solves a two-dimensional linear program subject to linear
        /// constraints defined by lines and a circular constraint.</summary>
        ///
        /// <param name="lines">Lines defining the linear constraints.</param>
        /// <param name="numObstLines">Count of obstacle lines.</param>
        /// <param name="beginLine">The line on which the 2-d linear program
        /// failed.</param>
        /// <param name="radius">The radius of the circular constraint.</param>
        /// <param name="result">A reference to the result of the linear program.
        /// </param>
        private void LinearProgram3(IList<Line> lines, int numObstLines, int beginLine, float radius, ref Vector2 result)
        {
            float distance = 0.0f;

            for (int i = beginLine; i < lines.Count; ++i)
            {
                if (RVOMath.Det(lines[i].Direction, lines[i].Point - result) > distance)
                {
                    /* Result does not satisfy constraint of line i. */
                    IList<Line> projLines = new List<Line>();
                    for (int ii = 0; ii < numObstLines; ++ii)
                    {
                        projLines.Add(lines[ii]);
                    }

                    for (int j = numObstLines; j < i; ++j)
                    {
                        Line line;

                        float determinant = RVOMath.Det(lines[i].Direction, lines[j].Direction);

                        if (MathF.Abs(determinant) <= RVOMath.RVO_EPSILON)
                        {
                            /* Line i and line j are parallel. */
                            if (lines[i].Direction * lines[j].Direction > 0.0f)
                            {
                                /* Line i and line j point in the same direction. */
                                continue;
                            }
                            else
                            {
                                /* Line i and line j point in opposite direction. */
                                line.Point = 0.5f * (lines[i].Point + lines[j].Point);
                            }
                        }
                        else
                        {
                            line.Point = lines[i].Point + (RVOMath.Det(lines[j].Direction, lines[i].Point - lines[j].Point) / determinant) * lines[i].Direction;
                        }

                        line.Direction = RVOMath.Normalize(lines[j].Direction - lines[i].Direction);
                        projLines.Add(line);
                    }

                    Vector2 tempResult = result;
                    if (LinearProgram2(projLines, radius, new Vector2(-lines[i].Direction.Y, lines[i].Direction.X), true, out result) < projLines.Count)
                    {
                        /*
                         * This should in principle not happen. The result is by
                         * definition already in the feasible region of this
                         * linear program. If it fails, it is due to small
                         * floating point error, and the current result is kept.
                         */
                        result = tempResult;
                    }

                    distance = RVOMath.Det(lines[i].Direction, lines[i].Point - result);
                }
            }
        }
    }
}
