/*
 * RVOMath.cs
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

namespace RVO
{
    /// <summary>Contains functions and constants used in multiple classes.
    /// </summary>
    public struct RVOMath
    {
        /// <summary>A sufficiently small positive number.</summary>
        internal const float RVO_EPSILON = 0.00001f;

        /// <summary>Computes the length of a specified two-dimensional vector.
        /// </summary>
        ///
        /// <param name="vector">The two-dimensional vector whose length is to be
        /// computed.</param>
        /// <returns>The length of the two-dimensional vector.</returns>
        public static float Abs(Vector2 vector)
        {
            return (float)Length(vector);
        }

        /// <summary>Computes the squared length of a specified two-dimensional
        /// vector.</summary>
        ///
        /// <returns>The squared length of the two-dimensional vector.</returns>
        ///
        /// <param name="vector">The two-dimensional vector whose squared length
        /// is to be computed.</param>
        public static float AbsSq(Vector2 vector)
        {
            return vector * vector;
        }

        /// <summary>Computes the normalization of the specified two-dimensional
        /// vector.</summary>
        ///
        /// <returns>The normalization of the two-dimensional vector.</returns>
        ///
        /// <param name="vector">The two-dimensional vector whose normalization
        /// is to be computed.</param>
        public static Vector2 Normalize(Vector2 vector)
        {
            double length = Length(vector);

            if (length <= RVO_EPSILON)
            {
                return new Vector2(0.0f, 0.0f);
            }

            return new Vector2((float)(vector.X / length), (float)(vector.Y / length));
        }

        private static double Length(Vector2 vector)
        {
            double x = vector.X;
            double y = vector.Y;
            return Math.Sqrt(x * x + y * y);
        }

        /// <summary>Computes the determinant of a two-dimensional square matrix
        /// with rows consisting of the specified two-dimensional vectors.
        /// </summary>
        ///
        /// <returns>The determinant of the two-dimensional square matrix.
        /// </returns>
        ///
        /// <param name="vector1">The top row of the two-dimensional square
        /// matrix.</param>
        /// <param name="vector2">The bottom row of the two-dimensional square
        /// matrix.</param>
        internal static float Det(Vector2 vector1, Vector2 vector2)
        {
            return vector1._x * vector2._y - vector1._y * vector2._x;
        }

        /// <summary>Computes the squared distance from a line segment with the
        /// specified endpoints to a specified point.</summary>
        ///
        /// <returns>The squared distance from the line segment to the point.
        /// </returns>
        ///
        /// <param name="vector1">The first endpoint of the line segment.</param>
        /// <param name="vector2">The second endpoint of the line segment.
        /// </param>
        /// <param name="vector3">The point to which the squared distance is to
        /// be calculated.</param>
        internal static float DistSqPointLineSegment(Vector2 vector1, Vector2 vector2, Vector2 vector3)
        {
            float lengthSq = AbsSq(vector2 - vector1);

            if (lengthSq <= RVO_EPSILON * RVO_EPSILON)
            {
                /* Degenerate segment: both endpoints are the same point. */
                return AbsSq(vector3 - vector1);
            }

            float r = ((vector3 - vector1) * (vector2 - vector1)) / lengthSq;

            if (r < 0.0f)
            {
                return AbsSq(vector3 - vector1);
            }

            if (r > 1.0f)
            {
                return AbsSq(vector3 - vector2);
            }

            return AbsSq(vector3 - (vector1 + r * (vector2 - vector1)));
        }

        /// <summary>Computes the signed distance from a line connecting the
        /// specified points to a specified point.</summary>
        ///
        /// <returns>Positive when the point c lies to the left of the line ab.
        /// </returns>
        ///
        /// <param name="a">The first point on the line.</param>
        /// <param name="b">The second point on the line.</param>
        /// <param name="c">The point to which the signed distance is to be
        /// calculated.</param>
        internal static float LeftOf(Vector2 a, Vector2 b, Vector2 c)
        {
            return Det(a - c, b - a);
        }

    }
}
