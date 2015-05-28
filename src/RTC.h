/*
 * RTC.h
 *
 *  Created on: 04.03.2015
 *      Author: sagok
 */

#ifndef RTC_H_
#define RTC_H_

#define RTC_ASYNCH_PREDIV  0x7F   /* LSE as RTC clock */
#define RTC_SYNCH_PREDIV   0x00FF /* LSE as RTC clock */

void Init_RTC(void);

void RTC_CalendarConfig(void);
void RTC_CalendarShow(uint8_t *showtime, uint8_t *showdate);
void RTC_GetTime(uint8_t *Hours, uint8_t *Minutes, uint8_t *Seconds);
void RTC_GetDate(uint8_t *Date, uint8_t *Month, uint8_t *Year);

#endif /* RTC_H_ */
