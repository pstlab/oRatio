/*
* Copyright (C) 2016 Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <limits>
#include <algorithm>

namespace ac {

	class interval {
	public:
		double lb;
		double ub;

		interval();
		interval(double value);
		interval(double lb, double ub);
		virtual ~interval();

		bool consistent() const;
		bool constant() const;
		bool intersecting(const interval& i) const;
		bool contains(const interval& bound) const;

		bool operator!=(const interval& right) const;
		bool operator<(const interval& right) const;
		bool operator<=(const interval& right) const;
		bool operator==(const interval& right) const;
		bool operator>=(const interval& right) const;
		bool operator>(const interval& right) const;

		friend interval operator&&(const interval& lhs, const interval& rhs) {
			return interval(std::max(lhs.lb, rhs.lb), std::min(lhs.ub, rhs.ub));
		}

		friend interval operator+(const interval& lhs, const interval& rhs) {
			return interval(lhs.lb + rhs.lb, lhs.ub + rhs.ub);
		}

		friend interval operator+(const interval& lhs, const double& rhs) {
			return interval(lhs.lb + rhs, lhs.ub + rhs);
		}

		friend interval operator+(const double& lhs, const interval& rhs) {
			return interval(lhs + rhs.lb, lhs + rhs.ub);
		}

		friend interval operator-(const interval& lhs, const interval& rhs) {
			return interval(lhs.lb - rhs.ub, lhs.ub - rhs.lb);
		}

		friend interval operator-(const interval& lhs, const double& rhs) {
			return interval(lhs.lb - rhs, lhs.ub - rhs);
		}

		friend interval operator-(const double& lhs, const interval& rhs) {
			return interval(lhs - rhs.ub, lhs - rhs.lb);
		}

		friend interval operator*(const interval& lhs, const interval& rhs) {
			interval result(std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity());
			for (const auto& i : { lhs.lb * rhs.lb, lhs.lb * rhs.ub, lhs.ub * rhs.lb, lhs.ub * rhs.ub }) {
				if (i < result.lb)
					result.lb = i;
				if (i > result.ub)
					result.ub = i;
			}
			return result;
		}

		friend interval operator*(const interval& lhs, const double& rhs) {
			if (rhs >= 0) {
				return interval(lhs.lb * rhs, lhs.ub * rhs);
			}
			else {
				return interval(lhs.ub * rhs, lhs.lb * rhs);
			}
		}

		friend interval operator*(const double& lhs, const interval& rhs) {
			if (lhs >= 0) {
				return interval(rhs.lb * lhs, rhs.ub * lhs);
			}
			else {
				return interval(rhs.ub * lhs, rhs.lb * lhs);
			}
		}

		friend interval operator/(const interval& lhs, const interval& rhs) {
			if (rhs.lb <= 0 && rhs.ub >= 0) {
				// 0 appears in the denominator..
				return interval();
			}
			else {
				interval result(std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity());
				for (const auto& i : { lhs.lb / rhs.lb, lhs.lb / rhs.ub, lhs.ub / rhs.lb, lhs.ub / rhs.ub }) {
					if (i < result.lb)
						result.lb = i;
					if (i > result.ub)
						result.ub = i;
				}
				return result;
			}
		}

		friend interval operator/(const interval& lhs, const double& rhs) {
			if (rhs >= 0) {
				return interval(lhs.lb / rhs, lhs.ub / rhs);
			}
			else {
				return interval(lhs.ub / rhs, lhs.lb / rhs);
			}
		}

		friend interval operator/(const double& lhs, const interval& rhs) {
			if (rhs.lb <= 0 && rhs.ub >= 0) {
				// 0 appears in the denominator..
				return interval();
			}
			else {
				interval result(std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity());
				for (const auto& i : { lhs / rhs.lb, lhs / rhs.ub }) {
					if (i < result.lb)
						result.lb = i;
					if (i > result.ub)
						result.ub = i;
				}
				return result;
			}
		}

		interval& operator+=(const interval& right);
		interval& operator+=(const double& right);
		interval& operator-=(const interval& right);
		interval& operator-=(const double& right);
		interval& operator*=(const interval& right);
		interval& operator*=(const double& right);
		interval& operator/=(const interval& right);
		interval& operator/=(const double& right);

		interval operator-() const;
	};
}


