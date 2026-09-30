#include <tk/tkernel.h>
#include <tm/tmonitor.h>

LOCAL void task_1(INT stacd, void *exinf);	// task execution function
LOCAL ID	tskid_1;			// Task ID number
LOCAL T_CTSK ctsk_1 = {				// Task creation information
	.itskpri	= 10,
	.stksz		= 1024,
	.task		= task_1,
	.tskatr		= TA_HLNG | TA_RNG3,
};

LOCAL void task_2(INT stacd, void *exinf);	// task execution function
LOCAL ID	tskid_2;			// Task ID number
LOCAL T_CTSK ctsk_2 = {				// Task creation information
	.itskpri	= 10,
	.stksz		= 1024,
	.task		= task_2,
	.tskatr		= TA_HLNG | TA_RNG3,
};

LOCAL void task_3(INT stacd, void *exinf);	// task execution function
LOCAL ID	tskid_3;			// Task ID number
LOCAL T_CTSK ctsk_3 = {				// Task creation information
	.itskpri	= 10,
	.stksz		= 1024,
	.task		= task_3,
	.tskatr		= TA_HLNG | TA_RNG3,
};

LOCAL void task_1(INT stacd, void *exinf)
{
	ID	dd = (ID)stacd;
	ER	ercd;
	UW	data;
	SZ	asz;

	while(1) {
		ercd = tk_srea_dev(dd, 10, &data, 1, &asz);
		if(ercd == E_OK) {
			tm_printf((UB*)"Data(A1) %d  ", data);
		} else {
			tm_printf((UB*)"Read Error (A1) %d\n", ercd);
		}

		ercd = tk_srea_dev(dd, 11, &data, 1, &asz);
		if(ercd == E_OK) {
			tm_printf((UB*)"Data(A2) %d  ", data);
		} else {
			tm_printf((UB*)"Read Error (A2) %d\n", ercd);
		}

		ercd = tk_srea_dev(dd, 13, &data, 1, &asz);
		if(ercd == E_OK) {
			tm_printf((UB*)"Data(A3) %d\n", data);
		} else {
			tm_printf((UB*)"Read Error (A3) %d\n", ercd);
		}

		tk_dly_tsk(1000);
	}
}

//LOCAL void task_2(INT stacd, void *exinf)
//{
//	ID	dd = (ID)stacd;
//	ER	ercd;
//	UW	data;
//	SZ	asz;
//
//	while(1) {
//		tk_dly_tsk(1500);
//	}
//}

LOCAL void task_3(INT stacd, void *exinf)
{
	ID	dd = (ID)stacd;
	ER	ercd;
	UW	data;
	SZ	asz;

	while(1) {
		ercd = tk_srea_dev(dd, 18, &data, 1, &asz);
		if(ercd == E_OK) {
			tm_printf((UB*)"Data(A0) %d\n", data);
		} else {
			tm_printf((UB*)"Read Error (A0) %d\n", ercd);
		}
		tk_dly_tsk(2000);
	}
}

/* usermain関数 */
EXPORT INT usermain(void)
{
	ID	dd1, dd2;

	tm_putstring((UB*)"Start User-main program.\n");

	dd1 = tk_opn_dev((UB*)"hadca", TD_READ);
	if(dd1 < E_OK) tm_printf((UB*)"Open Error %d\n", dd1);
	dd2 = tk_opn_dev((UB*)"hadcb", TD_READ);
	if(dd2 < E_OK) tm_printf((UB*)"Open Error %d\n", dd2);

	/* Create & Start Tasks */
	tskid_1 = tk_cre_tsk(&ctsk_1);
	tk_sta_tsk(tskid_1, (INT)dd1);

//	tskid_2 = tk_cre_tsk(&ctsk_2);
//	tk_sta_tsk(tskid_2, (INT)dd1);
//
	tskid_3 = tk_cre_tsk(&ctsk_3);
	tk_sta_tsk(tskid_3, (INT)dd2);

	tk_slp_tsk(TMO_FEVR);

	return 0;
}
