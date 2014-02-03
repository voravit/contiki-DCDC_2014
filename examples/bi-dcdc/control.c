/*
 * Copyright (c) 2013, KTH, Royal Institute of Technology(Stockholm, Sweden)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 * From the original control.c file
 * Author: Jorge Queroal
 * Created on: 2012-08-03
 *
 * Ported to contiki by: Voravit Tanyingyong
 *
 */

#include "contiki.h"
#include "lpc17xx.h"
#include "bang.h"
#include "control.h"
#include "adc.h"
#include "bang-config.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef ShowPeriodically
#include "comm.h"
#endif

int Vref, Imax, Vmax;
int Vout, Vin, Il;
int Vo, Vi, Io, Ii;
int i, Vom, Vim, Iom, Iim, Prio;

int Iref;
extern int ADC[4];
extern state_t CurrentState;
extern int UpdateChannel;

int user_allowed;

/* received and transmitted data buffers */
static uart_buffer_type uart_buf;
uint8_t welcome_msg[] = "\r";
static int echo = 0;

//char *state_strings[7] = {"BUCK_OFF","BUCK_SOFT","BUCK_ON","BOOST_OFF","BOOST_SOFT","BOOST_ON","DISCHARGE","ALL_OFF"};

void uart_buf_init(uart_buffer_type *buff)
{
        uint16_t i;
        i = 0;
        /* copy the welcome message to the transmit buffer */
        while (welcome_msg[i] != '\0')
        {
                buff->tx_buf[i] = welcome_msg[i];
                i++;
        }
        buff->rx_start_index = 0;
        buff->rx_len = 0;
        buff->tx_start_index = 0;
        buff->tx_len = i;
}


void break_float(float f, int *integer, int *decimal, int dlen, int base) {
        *integer = (int)f;
        f -= *integer;
        for(; dlen > 0; dlen--) {
                f *= base;
        }
        *decimal = (int)f;
//      *decimal = 0;
}


char* get_converter_state(void)
{
	/* [todo] new function to get on/off */
	static char *state_strings[8] = {"BUCK_OFF","BUCK_SOFT","BUCK_ON","BOOST_OFF","BOOST_SOFT","BOOST_ON","DISCHARGE","ALL_OFF"};
	if (CurrentState == 7)
		return "OFF";
	else
		return "ON";
//	return state_strings[CurrentState];
}

int set_ctrl_params(ctrl_params_t var, float value)
{
	int error=0;
	float tmp;
	switch (var)
	{
		case VREF:
			if ((value >= 0) && (value <= Vmax))
			{
				tmp = (value * 4095 * DIVIDEND) /(Vdd * DIVIDER);
				Vref = (int) tmp;
			}
			else
				error=1;
			break;
		case IMAX:
			if ((value >= 0) && (value <= 6))
			{
				tmp = (value * 4095 * 0.151) / (Vdd * 2);
				Imax = (int) tmp;
			}
			else
				error=1;
			break;
		case VMAX:
			if ((value >= 0) && (value <= 30))
			{
				tmp = (value * 4095 * DIVIDEND) / (Vdd * DIVIDER);
				Vmax = (int) tmp;
			}
			else
				error=1;
			break;
		case PRIO_REF:
			if ((value >= 5) && (value <= 25))
			{
//				Prio = (int) value;
				Prio = (int)(value * 4095 * DIVIDEND) /(Vdd * DIVIDER);
			}
			else
				error=1;
			break;
	}
	return error;
}

float get_ctrl_params(ctrl_params_t var)
{
	float result=0;
	switch (var)
	{
		case VREF:
			result = (Vref * Vdd * DIVIDER) / (4095 * DIVIDEND);
			break;
		case IMAX:
			result = (Imax * Vdd * 2) / (4095 * 0.151);
			break;
		case VMAX:
			result = (Vmax * Vdd * DIVIDER) / (4095 * DIVIDEND);
			break;
		case PRIO_REF:
			result = (int)(Prio * Vdd * DIVIDER) / (4095 * DIVIDEND);
			break;
	}
	return result;

}

float get_svector(svector_t var)
{
	float result=0;
	switch (var)
	{
		case VOUT:
			result = (Vo * Vdd * DIVIDER) / (4095 * DIVIDEND);
			break;
		case VIN:
			result = (Vi * Vdd * DIVIDER) / (4095 * DIVIDEND);
			break;
		case IOUT:
			result = (Io * Vdd * 2) / (4095 * 0.151);
			break;
		case IIN:
			result = (Ii * Vdd * 2) / (4095 * 0.151);
			break;
		case PRIO:
			result = (int)((Prio * Vdd * DIVIDER) / (4095 * DIVIDEND));
			break;
	}
	return result;

}

void ValueInit(void)
{
	Vref = 0;
	Imax = (int)(I_IMAX * 4095 * 0.151) /(Vdd * 2);
	Vmax = (int)(I_VMAX * 4095 * DIVIDEND) /(Vdd * DIVIDER);
	Vo = 0;
	Vi = 0;
	Io = 0;
	Ii = 0;
	i = 1;
	Vom = 0;
	Vim = 0;
	Iom = 0;
	Iim = 0;
	user_allowed = 0;
	uart_buf_init(&uart_buf);
}

void MeanValues(void)
{
	float aux;

	i++;
	Vom += Vout;
	Vim += Vin;
	if((LPC_GPIO2->FIOPIN & 0x0008) || (CurrentState == BOOST_SOFT))
		Iom += Il;
	if(LPC_GPIO2->FIOPIN & 0x0002)
		Iim += Il;

	if((i % 100000) == 0)
	{
		Vo = Vom / 100000;
		Vi = Vim / 100000;
		Io = Iom / 100000;
		Ii = Iim / 100000;
		Vom = 0;
		Vim = 0;
		Iom = 0;
		Iim = 0;
	}
#ifdef ShowPeriodically
	if(i >= 100000)
	{
		i = 1;
		print("\r\n Vo:");
		aux = (Vo * Vdd * DIVIDER) / (4095 * DIVIDEND);
		print_float(&aux);
		print(" Vin:");
		aux = (Vi * Vdd * DIVIDER) / (4095 * DIVIDEND);
		print_float(&aux);
		print(" Io:");
		aux = (Io * Vdd * 2) / (4095 * 0.151);
		print_float(&aux);
		print(" Ii:");
		aux = (Ii * Vdd * 2) / (4095 * 0.151);
		print_float(&aux);
	}
#endif
}

void BangBang(void)
{
	if(Vref > 0 && Prio <= Vi)
	{
		/* Safe maximum voltage limitation (it uses configuration number4) */
		if (Vout > Vmax)
			SetState(DISCHARGE);
		/* Current limitation */
		else if(Il > Imax)
		{
			/* If output is over input it's possible to limit the current with state 2 */
			if(Vout > Vin)
				SetState(BOOST_OFF);
			/* else it must be limited with state 1 */
			else
				SetState(BUCK_OFF);
		}
		else
		{
			/* To make the decision of which converter must be used, it's necessary to check
			 * the relationship between Vin and Vref but it's also necessary to know if the
			 * buck converter is in the low conversion efficiency zone. Check operating zone
			 * function in software. */

			/* Boost single converter */
//			if(operatingzone(Vin, Vref, Vout))
			if(Vref > Vin)
			{
				/* OFF mode (state 2 for boost converter) */
				if (Vout > Vref)
				{
					if(Il > 10)
						SetState(BOOST_OFF);
					else
						SetState(BOOST_SOFT);
				}
				/* ON mode (state 3 for boost converter) */
				else
					SetState(BOOST_ON);
			}
			/* Buck single converter */
			else
			{
				/* OFF mode (state 1 for buck converter) */
				if (Vout > Vref)
				{
					if(Il > 10)
						SetState(BUCK_OFF);
					else
						SetState(BUCK_SOFT);
				}
				/* ON mode (state 2 for buck converter) */
				else
					SetState(BUCK_ON);
			}
		}
	}
	else
		SetState(ALL_OFF);
}


