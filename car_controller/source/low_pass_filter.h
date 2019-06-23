/*
 * low_pass_filter.h
 *
 *  Created on: Jun 23, 2019
 *      Author: djaenicke
 */

#ifndef LOW_PASS_FILTER_H_
#define LOW_PASS_FILTER_H_

template <class T>
T LP_Filter(T x, T y_n_1, float alpha)
{
	T y;

	y = ((1-alpha)*y_n_1)+(x*alpha);

	return y;
}

#endif /* LOW_PASS_FILTER_H_ */
