/*
 * KdTree.cs
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
    /// <summary>Defines k-D trees for agents and static obstacles in the
    /// simulation.</summary>
    internal class KdTree
    {
        /// <summary>Defines a node of an agent k-D tree.</summary>
        private struct AgentTreeNode
        {
            internal int _begin;
            internal int _end;
            internal int _left;
            internal int _right;
            internal float _maxX;
            internal float _maxY;
            internal float _minX;
            internal float _minY;
        }

        /// <summary>Defines a pair of scalar values.</summary>
        private struct FloatPair
        {
            private readonly float _a;
            private readonly float _b;

            /// <summary>Constructs and initializes a pair of scalar
            /// values.</summary>
            ///
            /// <param name="a">The first scalar value.</param>
            /// <param name="b">The second scalar value.</param>
            internal FloatPair(float a, float b)
            {
                _a = a;
                _b = b;
            }

            /// <summary>Returns true if the first pair of scalar values is less
            /// than the second pair of scalar values.</summary>
            ///
            /// <returns>True if the first pair of scalar values is less than the
            /// second pair of scalar values.</returns>
            ///
            /// <param name="pair1">The first pair of scalar values.</param>
            /// <param name="pair2">The second pair of scalar values.</param>
            public static bool operator <(FloatPair pair1, FloatPair pair2)
            {
                return pair1._a < pair2._a || pair1._a == pair2._a && pair1._b < pair2._b;
            }

            /// <summary>Returns true if the first pair of scalar values is less
            /// than or equal to the second pair of scalar values.</summary>
            ///
            /// <returns>True if the first pair of scalar values is less than or
            /// equal to the second pair of scalar values.</returns>
            ///
            /// <param name="pair1">The first pair of scalar values.</param>
            /// <param name="pair2">The second pair of scalar values.</param>
            public static bool operator <=(FloatPair pair1, FloatPair pair2)
            {
                return (pair1._a == pair2._a && pair1._b == pair2._b) || pair1 < pair2;
            }

            /// <summary>Returns true if the first pair of scalar values is
            /// greater than the second pair of scalar values.</summary>
            ///
            /// <returns>True if the first pair of scalar values is greater than
            /// the second pair of scalar values.</returns>
            ///
            /// <param name="pair1">The first pair of scalar values.</param>
            /// <param name="pair2">The second pair of scalar values.</param>
            public static bool operator >(FloatPair pair1, FloatPair pair2)
            {
                return !(pair1 <= pair2);
            }

            /// <summary>Returns true if the first pair of scalar values is
            /// greater than or equal to the second pair of scalar values.
            /// </summary>
            ///
            /// <returns>True if the first pair of scalar values is greater than
            /// or equal to the second pair of scalar values.</returns>
            ///
            /// <param name="pair1">The first pair of scalar values.</param>
            /// <param name="pair2">The second pair of scalar values.</param>
            public static bool operator >=(FloatPair pair1, FloatPair pair2)
            {
                return !(pair1 < pair2);
            }
        }

        /// <summary>Defines a node of an obstacle k-D tree.</summary>
        private class ObstacleTreeNode
        {
            internal Obstacle _obstacle;
            internal ObstacleTreeNode _left;
            internal ObstacleTreeNode _right;
        };

        /// <summary>The maximum size of an agent k-D tree leaf.</summary>
        /* Empirically chosen; balances tree depth against per-leaf work. */
        private const int MAX_LEAF_SIZE = 10;

        private Agent[] _agents;
        private AgentTreeNode[] _agentTree;
        private ObstacleTreeNode _obstacleTree;

        /// <summary>Builds an agent k-D tree.</summary>
        internal void BuildAgentTree()
        {
            if (_agents is null || _agents.Length != Simulator.Instance._agents.Count)
            {
                _agents = new Agent[Simulator.Instance._agents.Count];

                for (int i = 0; i < _agents.Length; ++i)
                {
                    _agents[i] = Simulator.Instance._agents[i];
                }

                _agentTree = new AgentTreeNode[2 * _agents.Length];

                for (int i = 0; i < _agentTree.Length; ++i)
                {
                    _agentTree[i] = new AgentTreeNode();
                }
            }

            if (_agents.Length != 0)
            {
                BuildAgentTreeRecursive(0, _agents.Length, 0);
            }
        }

        /// <summary>Builds an obstacle k-D tree.</summary>
        internal void BuildObstacleTree()
        {
            _obstacleTree = new ObstacleTreeNode();

            IList<Obstacle> obstacles = new List<Obstacle>(Simulator.Instance._obstacles.Count);

            for (int i = 0; i < Simulator.Instance._obstacles.Count; ++i)
            {
                obstacles.Add(Simulator.Instance._obstacles[i]);
            }

            _obstacleTree = BuildObstacleTreeRecursive(obstacles);
        }

        /// <summary>Computes the agent neighbors of the specified agent.
        /// </summary>
        ///
        /// <param name="agent">The agent for which agent neighbors are to be
        /// computed.</param>
        /// <param name="rangeSq">The squared range around the agent.</param>
        internal void ComputeAgentNeighbors(Agent agent, ref float rangeSq)
        {
            QueryAgentTreeRecursive(agent, ref rangeSq, 0);
        }

        /// <summary>Computes the obstacle neighbors of the specified agent.
        /// </summary>
        ///
        /// <param name="agent">The agent for which obstacle neighbors are to be
        /// computed.</param>
        /// <param name="rangeSq">The squared range around the agent.</param>
        internal void ComputeObstacleNeighbors(Agent agent, float rangeSq)
        {
            QueryObstacleTreeRecursive(agent, rangeSq, _obstacleTree);
        }

        /// <summary>Queries the visibility between two points within a specified
        /// radius.</summary>
        ///
        /// <returns>True if q1 and q2 are mutually visible within the radius;
        /// false otherwise.</returns>
        ///
        /// <param name="q1">The first point between which visibility is to be
        /// tested.</param>
        /// <param name="q2">The second point between which visibility is to be
        /// tested.</param>
        /// <param name="radius">The radius within which visibility is to be
        /// tested.</param>
        internal bool QueryVisibility(Vector2 q1, Vector2 q2, float radius)
        {
            return QueryVisibilityRecursive(q1, q2, radius, _obstacleTree);
        }

        /// <summary>Recursive method for building an agent k-D tree.</summary>
        ///
        /// <param name="begin">The beginning agent k-D tree node node index.
        /// </param>
        /// <param name="end">The ending agent k-D tree node index.</param>
        /// <param name="node">The current agent k-D tree node index.</param>
        private void BuildAgentTreeRecursive(int begin, int end, int node)
        {
            _agentTree[node]._begin = begin;
            _agentTree[node]._end = end;
            _agentTree[node]._minX = _agentTree[node]._maxX = _agents[begin]._position._x;
            _agentTree[node]._minY = _agentTree[node]._maxY = _agents[begin]._position._y;

            for (int i = begin + 1; i < end; ++i)
            {
                _agentTree[node]._maxX = Math.Max(_agentTree[node]._maxX, _agents[i]._position._x);
                _agentTree[node]._minX = Math.Min(_agentTree[node]._minX, _agents[i]._position._x);
                _agentTree[node]._maxY = Math.Max(_agentTree[node]._maxY, _agents[i]._position._y);
                _agentTree[node]._minY = Math.Min(_agentTree[node]._minY, _agents[i]._position._y);
            }

            if (end - begin > MAX_LEAF_SIZE)
            {
                /* No leaf node. */
                bool isVertical = _agentTree[node]._maxX - _agentTree[node]._minX > _agentTree[node]._maxY - _agentTree[node]._minY;
                float splitValue = 0.5f * (isVertical ? _agentTree[node]._maxX + _agentTree[node]._minX : _agentTree[node]._maxY + _agentTree[node]._minY);

                int left = begin;
                int right = end;

                while (left < right)
                {
                    while (left < right && (isVertical ? _agents[left]._position._x : _agents[left]._position._y) < splitValue)
                    {
                        ++left;
                    }

                    while (right > left && (isVertical ? _agents[right - 1]._position._x : _agents[right - 1]._position._y) >= splitValue)
                    {
                        --right;
                    }

                    if (left < right)
                    {
                        Agent tempAgent = _agents[left];
                        _agents[left] = _agents[right - 1];
                        _agents[right - 1] = tempAgent;
                        ++left;
                        --right;
                    }
                }

                int leftSize = left - begin;

                if (leftSize == 0)
                {
                    ++leftSize;
                    ++left;
                }

                _agentTree[node]._left = node + 1;
                _agentTree[node]._right = node + 2 * leftSize;

                BuildAgentTreeRecursive(begin, left, _agentTree[node]._left);
                BuildAgentTreeRecursive(left, end, _agentTree[node]._right);
            }
        }

        /// <summary>Recursive method for building an obstacle k-D tree.
        /// </summary>
        ///
        /// <returns>An obstacle k-D tree node.</returns>
        ///
        /// <param name="obstacles">A list of obstacles.</param>
        private ObstacleTreeNode BuildObstacleTreeRecursive(IList<Obstacle> obstacles)
        {
            if (obstacles.Count == 0)
            {
                return null;
            }

            ObstacleTreeNode node = new();

            int optimalSplit = 0;
            int minLeft = obstacles.Count;
            int minRight = obstacles.Count;

            for (int i = 0; i < obstacles.Count; ++i)
            {
                int leftSize = 0;
                int rightSize = 0;

                Obstacle obstacleI1 = obstacles[i];
                Obstacle obstacleI2 = obstacleI1._next;

                /* Compute optimal split node. */
                for (int j = 0; j < obstacles.Count; ++j)
                {
                    if (i == j)
                    {
                        continue;
                    }

                    Obstacle obstacleJ1 = obstacles[j];
                    Obstacle obstacleJ2 = obstacleJ1._next;

                    float j1LeftOfI = RVOMath.LeftOf(obstacleI1._point, obstacleI2._point, obstacleJ1._point);
                    float j2LeftOfI = RVOMath.LeftOf(obstacleI1._point, obstacleI2._point, obstacleJ2._point);

                    if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON)
                    {
                        ++leftSize;
                    }
                    else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON)
                    {
                        ++rightSize;
                    }
                    else
                    {
                        ++leftSize;
                        ++rightSize;
                    }

                    if (new FloatPair(Math.Max(leftSize, rightSize), Math.Min(leftSize, rightSize)) >= new FloatPair(Math.Max(minLeft, minRight), Math.Min(minLeft, minRight)))
                    {
                        break;
                    }
                }

                if (new FloatPair(Math.Max(leftSize, rightSize), Math.Min(leftSize, rightSize)) < new FloatPair(Math.Max(minLeft, minRight), Math.Min(minLeft, minRight)))
                {
                    minLeft = leftSize;
                    minRight = rightSize;
                    optimalSplit = i;
                }
            }

            {
                /* Build split node. */
                IList<Obstacle> leftObstacles = new List<Obstacle>(minLeft);

                for (int n = 0; n < minLeft; ++n)
                {
                    leftObstacles.Add(null);
                }

                IList<Obstacle> rightObstacles = new List<Obstacle>(minRight);

                for (int n = 0; n < minRight; ++n)
                {
                    rightObstacles.Add(null);
                }

                int leftCounter = 0;
                int rightCounter = 0;
                int i = optimalSplit;

                Obstacle obstacleI1 = obstacles[i];
                Obstacle obstacleI2 = obstacleI1._next;

                for (int j = 0; j < obstacles.Count; ++j)
                {
                    if (i == j)
                    {
                        continue;
                    }

                    Obstacle obstacleJ1 = obstacles[j];
                    Obstacle obstacleJ2 = obstacleJ1._next;

                    float j1LeftOfI = RVOMath.LeftOf(obstacleI1._point, obstacleI2._point, obstacleJ1._point);
                    float j2LeftOfI = RVOMath.LeftOf(obstacleI1._point, obstacleI2._point, obstacleJ2._point);

                    if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON)
                    {
                        leftObstacles[leftCounter++] = obstacles[j];
                    }
                    else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON)
                    {
                        rightObstacles[rightCounter++] = obstacles[j];
                    }
                    else
                    {
                        /* Split obstacle j. */
                        float t = RVOMath.Det(obstacleI2._point - obstacleI1._point, obstacleJ1._point - obstacleI1._point) / RVOMath.Det(obstacleI2._point - obstacleI1._point, obstacleJ1._point - obstacleJ2._point);

                        Vector2 splitPoint = obstacleJ1._point + t * (obstacleJ2._point - obstacleJ1._point);

                        Obstacle newObstacle = new();
                        newObstacle._point = splitPoint;
                        newObstacle._previous = obstacleJ1;
                        newObstacle._next = obstacleJ2;
                        newObstacle._convex = true;
                        newObstacle._direction = obstacleJ1._direction;

                        newObstacle._id = Simulator.Instance._obstacles.Count;

                        Simulator.Instance._obstacles.Add(newObstacle);

                        obstacleJ1._next = newObstacle;
                        obstacleJ2._previous = newObstacle;

                        if (j1LeftOfI > 0.0f)
                        {
                            leftObstacles[leftCounter++] = obstacleJ1;
                            rightObstacles[rightCounter++] = newObstacle;
                        }
                        else
                        {
                            rightObstacles[rightCounter++] = obstacleJ1;
                            leftObstacles[leftCounter++] = newObstacle;
                        }
                    }
                }

                node._obstacle = obstacleI1;
                node._left = BuildObstacleTreeRecursive(leftObstacles);
                node._right = BuildObstacleTreeRecursive(rightObstacles);

                return node;
            }
        }

        /// <summary>Recursive method for computing the agent neighbors of the
        /// specified agent.</summary>
        ///
        /// <param name="agent">The agent for which agent neighbors are to be
        /// computed.</param>
        /// <param name="rangeSq">The squared range around the agent.</param>
        /// <param name="node">The current agent k-D tree node index.</param>
        private void QueryAgentTreeRecursive(Agent agent, ref float rangeSq, int node)
        {
            if (_agentTree[node]._end - _agentTree[node]._begin <= MAX_LEAF_SIZE)
            {
                for (int i = _agentTree[node]._begin; i < _agentTree[node]._end; ++i)
                {
                    agent.InsertAgentNeighbor(_agents[i], ref rangeSq);
                }
            }
            else
            {
                int leftNode = _agentTree[node]._left;
                float leftDx = Math.Max(0.0f, _agentTree[leftNode]._minX - agent._position._x) + Math.Max(0.0f, agent._position._x - _agentTree[leftNode]._maxX);
                float leftDy = Math.Max(0.0f, _agentTree[leftNode]._minY - agent._position._y) + Math.Max(0.0f, agent._position._y - _agentTree[leftNode]._maxY);
                float distSqLeft = leftDx * leftDx + leftDy * leftDy;

                int rightNode = _agentTree[node]._right;
                float rightDx = Math.Max(0.0f, _agentTree[rightNode]._minX - agent._position._x) + Math.Max(0.0f, agent._position._x - _agentTree[rightNode]._maxX);
                float rightDy = Math.Max(0.0f, _agentTree[rightNode]._minY - agent._position._y) + Math.Max(0.0f, agent._position._y - _agentTree[rightNode]._maxY);
                float distSqRight = rightDx * rightDx + rightDy * rightDy;

                if (distSqLeft < distSqRight)
                {
                    if (distSqLeft < rangeSq)
                    {
                        QueryAgentTreeRecursive(agent, ref rangeSq, leftNode);

                        if (distSqRight < rangeSq)
                        {
                            QueryAgentTreeRecursive(agent, ref rangeSq, rightNode);
                        }
                    }
                }
                else
                {
                    if (distSqRight < rangeSq)
                    {
                        QueryAgentTreeRecursive(agent, ref rangeSq, rightNode);

                        if (distSqLeft < rangeSq)
                        {
                            QueryAgentTreeRecursive(agent, ref rangeSq, leftNode);
                        }
                    }
                }

            }
        }

        /// <summary>Recursive method for computing the obstacle neighbors of the
        /// specified agent.</summary>
        ///
        /// <param name="agent">The agent for which obstacle neighbors are to be
        /// computed.</param>
        /// <param name="rangeSq">The squared range around the agent.</param>
        /// <param name="node">The current obstacle k-D node.</param>
        private void QueryObstacleTreeRecursive(Agent agent, float rangeSq, ObstacleTreeNode node)
        {
            if (node is not null)
            {
                Obstacle obstacle1 = node._obstacle;
                Obstacle obstacle2 = obstacle1._next;

                float agentLeftOfLine = RVOMath.LeftOf(obstacle1._point, obstacle2._point, agent._position);

                QueryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0.0f ? node._left : node._right);

                float distSqLine = agentLeftOfLine * agentLeftOfLine / RVOMath.AbsSq(obstacle2._point - obstacle1._point);

                if (distSqLine < rangeSq)
                {
                    if (agentLeftOfLine < 0.0f)
                    {
                        /*
                         * Try obstacle at this node only if agent is on right side of
                         * obstacle (and can see obstacle).
                         */
                        agent.InsertObstacleNeighbor(node._obstacle, rangeSq);
                    }

                    /* Try other side of line. */
                    QueryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0.0f ? node._right : node._left);
                }
            }
        }

        /// <summary>Recursive method for querying the visibility between two
        /// points within a specified radius.</summary>
        ///
        /// <returns>True if q1 and q2 are mutually visible within the radius;
        /// false otherwise.</returns>
        ///
        /// <param name="q1">The first point between which visibility is to be
        /// tested.</param>
        /// <param name="q2">The second point between which visibility is to be
        /// tested.</param>
        /// <param name="radius">The radius within which visibility is to be
        /// tested.</param>
        /// <param name="node">The current obstacle k-D node.</param>
        private bool QueryVisibilityRecursive(Vector2 q1, Vector2 q2, float radius, ObstacleTreeNode node)
        {
            if (node is null)
            {
                return true;
            }

            Obstacle obstacle1 = node._obstacle;
            Obstacle obstacle2 = obstacle1._next;

            double q1LeftOfI = VisibilitySide(obstacle1._point, obstacle2._point, q1);
            double q2LeftOfI = VisibilitySide(obstacle1._point, obstacle2._point, q2);
            double invLengthI = 1.0 / VisibilityLengthSq(obstacle1._point, obstacle2._point);
            double radiusSq = (double)radius * radius;

            if (q1LeftOfI >= 0.0f && q2LeftOfI >= 0.0f)
            {
                return QueryVisibilityRecursive(q1, q2, radius, node._left) && ((q1LeftOfI * q1LeftOfI * invLengthI >= radiusSq && q2LeftOfI * q2LeftOfI * invLengthI >= radiusSq) || QueryVisibilityRecursive(q1, q2, radius, node._right));
            }

            if (q1LeftOfI <= 0.0f && q2LeftOfI <= 0.0f)
            {
                return QueryVisibilityRecursive(q1, q2, radius, node._right) && ((q1LeftOfI * q1LeftOfI * invLengthI >= radiusSq && q2LeftOfI * q2LeftOfI * invLengthI >= radiusSq) || QueryVisibilityRecursive(q1, q2, radius, node._left));
            }

            if (q1LeftOfI >= 0.0f && q2LeftOfI <= 0.0f)
            {
                /* One can see through obstacle from left to right. */
                return QueryVisibilityRecursive(q1, q2, radius, node._left) && QueryVisibilityRecursive(q1, q2, radius, node._right);
            }

            double point1LeftOfQ = VisibilitySide(q1, q2, obstacle1._point);
            double point2LeftOfQ = VisibilitySide(q1, q2, obstacle2._point);
            double invLengthQ = 1.0 / VisibilityLengthSq(q1, q2);

            return point1LeftOfQ * point2LeftOfQ >= 0.0f && point1LeftOfQ * point1LeftOfQ * invLengthQ > radiusSq && point2LeftOfQ * point2LeftOfQ * invLengthQ > radiusSq && QueryVisibilityRecursive(q1, q2, radius, node._left) && QueryVisibilityRecursive(q1, q2, radius, node._right);
        }

        private static double VisibilitySide(Vector2 a, Vector2 b, Vector2 point)
        {
            double x = (double)b.X - a.X;
            double y = (double)b.Y - a.Y;
            return x * ((double)point.Y - a.Y) - y * ((double)point.X - a.X);
        }

        private static double VisibilityLengthSq(Vector2 a, Vector2 b)
        {
            double x = (double)b.X - a.X;
            double y = (double)b.Y - a.Y;
            return x * x + y * y;
        }
    }
}
