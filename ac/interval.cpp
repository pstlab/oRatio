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

#include "interval.h"

using namespace ac;

interval::interval() : lb(-std::numeric_limits<double>::infinity()), ub(std::numeric_limits<double>::infinity()) {}

interval::interval(double value) : lb(value), ub(value) {}

interval::interval(double lb, double ub) : lb(lb), ub(ub) {}

interval::~interval() {}

bool interval::consistent() const {
	return lb <= ub;
}

bool interval::constant() const {
	return lb == ub;
}

bool interval::intersecting(const interval& i) const {
	return ub >= i.lb && lb <= i.ub;
}

bool interval::contains(const interval& i) const {
	return lb <= i.lb && ub >= i.ub;
}

bool interval::operator!=(const interval& right) const {
	return ub < right.lb || lb > right.ub;
}

bool interval::operator<(const interval& right) const {
	return ub < right.lb;
}

bool interval::operator<=(const interval& right) const {
	return ub <= right.lb;
}

bool interval::operator==(const interval& right) const {
	return constant() && right.constant() && lb == right.lb;
}

bool interval::operator>=(const interval& right) const {
	return lb >= right.ub;
}

bool interval::operator>(const interval& right) const {
	return lb > right.ub;
}

interval& interval::operator+=(const interval& right) {
	lb += right.lb;
	ub += right.ub;
	return *this;
}

interval& interval::operator+=(const double& right) {
	lb += right;
	ub += right;
	return *this;
}

interval& interval::operator-=(const interval& right) {
	lb -= right.ub;
	ub -= right.lb;
	return *this;
}

interval& interval::operator-=(const double& right) {
	lb -= right;
	ub -= right;
	return *this;
}

interval& interval::operator*=(const interval& right) {
	double c_lb = std::numeric_limits<double>::infinity();
	double c_ub = -std::numeric_limits<double>::infinity();
	for (const auto& i : { lb * right.lb, lb * right.ub, ub * right.lb, ub * right.ub }) {
		if (i < c_lb)
			c_lb = i;
		if (i > c_ub)
			c_ub = i;
	}
	lb = c_lb;
	ub = c_ub;
	return *this;
}

interval& interval::operator*=(const double& right) {
	if (right >= 0) {
		lb *= right;
		ub *= right;
	}
	else {
		double c_lb = lb;
		lb = ub * right;
		ub = c_lb * right;
	}
	return *this;
}

interval& interval::operator/=(const interval& right) {
	if (right.lb <= 0 && right.ub >= 0) {
		// 0 appears in the denominator..
		lb = -std::numeric_limits<double>::infinity();
		ub = std::numeric_limits<double>::infinity();
	}
	else {
		double c_lb = std::numeric_limits<double>::infinity();
		double c_ub = -std::numeric_limits<double>::infinity();
		for (const auto& i : { lb / right.lb, lb / right.ub, ub / right.lb, ub / right.ub }) {
			if (i < c_lb)
				c_lb = i;
			if (i > c_ub)
				c_ub = i;
		}
		lb = c_lb;
		ub = c_ub;
	}
	return *this;
}

interval& interval::operator/=(const double& right) {
	if (right >= 0) {
		lb /= right;
		ub /= right;
	}
	else {
		double c_lb = lb;
		lb = ub / right;
		ub = c_lb / right;
	}
	return *this;
}

interval interval::operator-() const {
	return interval(-ub, -lb);
}