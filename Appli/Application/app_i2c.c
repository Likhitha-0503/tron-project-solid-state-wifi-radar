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

/* 温度湿度センサー(SHT35)情報定義*/
#define	I2C_SADR	(0x45)		// I2Cターゲット・アドレス	

LOCAL void task_1(INT stacd, void *exinf)
{
	ID	dd_i2c = (ID)stacd;	// I2Cデバイス・ディスクリプタ

	const UB	cmd[2] = {0x2C, 0x06}; 	// 測定コマンド
	UB		data[6];		// 測定データ
	SZ		asz;			// リードサイズ
	ER		err;			// エラーコード

	UW		temp, humi;		// 測定データ

	while(1) {
		/* ① 測定コマンドの送信 */
		err = tk_swri_dev(dd_i2c, I2C_SADR, cmd, sizeof(cmd), &asz);
		if(err < E_OK) {
			tm_printf((UB*)"Send err = %d\n", err);
		}
		tk_dly_tsk(10);

		/* ② データの受信 */
		err = tk_srea_dev(dd_i2c, I2C_SADR, data, sizeof(data), &asz);
		if(err < E_OK) {
			tm_printf((UB*)"Receive err = %d\n", err);
		} else {
			temp = ((((UW)data[0]<<8) | data[1]) *17500 >> 16) - 4500;
			humi = (((UW)data[3]<<8) | data[4])*10000 >> 16;

			tm_printf((UB*)"TEMP %d  HUMI %d\n", temp, humi);
		}
		tk_dly_tsk(1000);			// ⑤ タスクの実行待ち
	}
	tk_ext_tsk();
}

/* usermain関数 */
EXPORT INT usermain(void)
{
	ID	dd;

	tm_putstring((UB*)"Start User-main program.\n");

	dd = tk_opn_dev((UB*)"hiica", TD_UPDATE);
	if(dd < E_OK) tm_printf((UB*)"Open Error %d\n", dd);

	/* Create & Start Tasks */
	tskid_1 = tk_cre_tsk(&ctsk_1);
	tk_sta_tsk(tskid_1, dd);

	tk_slp_tsk(TMO_FEVR);

	return 0;
}
