
d:\mt4\backups\mt4_flash.elf:     file format elf32-avr


Disassembly of section .data:

00008e92 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x8e92>:
    8e92:	1f 92       	push	r1
    8e94:	0f 92       	push	r0
    8e96:	0f b6       	in	r0, 0x3f	; 63
    8e98:	0f 92       	push	r0
    8e9a:	11 24       	eor	r1, r1
    8e9c:	0b b6       	in	r0, 0x3b	; 59
    8e9e:	0f 92       	push	r0
    8ea0:	ef 92       	push	r14
    8ea2:	ff 92       	push	r15
    8ea4:	0f 93       	push	r16
    8ea6:	1f 93       	push	r17
    8ea8:	2f 93       	push	r18
    8eaa:	3f 93       	push	r19
    8eac:	4f 93       	push	r20
    8eae:	5f 93       	push	r21
    8eb0:	6f 93       	push	r22
    8eb2:	7f 93       	push	r23
    8eb4:	8f 93       	push	r24
    8eb6:	9f 93       	push	r25
    8eb8:	af 93       	push	r26
    8eba:	bf 93       	push	r27
    8ebc:	cf 93       	push	r28
    8ebe:	df 93       	push	r29
    8ec0:	ef 93       	push	r30
    8ec2:	ff 93       	push	r31
    8ec4:	80 91 9c 0a 	lds	r24, 0x0A9C	; 0x800a9c <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d7e>
    8ec8:	81 11       	cpse	r24, r1
    8eca:	aa c3       	rjmp	.+1876   	; 0x9620 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9620>
    8ecc:	22 b1       	in	r18, 0x02	; 2
    8ece:	80 91 bc 0a 	lds	r24, 0x0ABC	; 0x800abc <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9e>
    8ed2:	98 2f       	mov	r25, r24
    8ed4:	95 75       	andi	r25, 0x55	; 85
    8ed6:	2a 7a       	andi	r18, 0xAA	; 170
    8ed8:	92 2b       	or	r25, r18
    8eda:	92 b9       	out	0x02, r25	; 2
    8edc:	98 b1       	in	r25, 0x08	; 8
    8ede:	8a 72       	andi	r24, 0x2A	; 42
    8ee0:	95 7d       	andi	r25, 0xD5	; 213
    8ee2:	89 2b       	or	r24, r25
    8ee4:	88 b9       	out	0x08, r24	; 8
    8ee6:	22 b1       	in	r18, 0x02	; 2
    8ee8:	80 91 bb 0a 	lds	r24, 0x0ABB	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    8eec:	98 2f       	mov	r25, r24
    8eee:	9a 7a       	andi	r25, 0xAA	; 170
    8ef0:	25 75       	andi	r18, 0x55	; 85
    8ef2:	92 2b       	or	r25, r18
    8ef4:	92 b9       	out	0x02, r25	; 2
    8ef6:	98 b1       	in	r25, 0x08	; 8
    8ef8:	84 75       	andi	r24, 0x54	; 84
    8efa:	9b 7a       	andi	r25, 0xAB	; 171
    8efc:	89 2b       	or	r24, r25
    8efe:	88 b9       	out	0x08, r24	; 8
    8f00:	80 91 ba 0a 	lds	r24, 0x0ABA	; 0x800aba <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9c>
    8f04:	86 bd       	out	0x26, r24	; 38
    8f06:	82 e0       	ldi	r24, 0x02	; 2
    8f08:	85 bd       	out	0x25, r24	; 37
    8f0a:	81 e0       	ldi	r24, 0x01	; 1
    8f0c:	80 93 9c 0a 	sts	0x0A9C, r24	; 0x800a9c <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d7e>
    8f10:	78 94       	sei
    8f12:	80 91 de 0a 	lds	r24, 0x0ADE	; 0x800ade <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dc0>
    8f16:	90 91 df 0a 	lds	r25, 0x0ADF	; 0x800adf <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dc1>
    8f1a:	89 2b       	or	r24, r25
    8f1c:	09 f0       	breq	.+2      	; 0x8f20 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x8f20>
    8f1e:	12 c1       	rjmp	.+548    	; 0x9144 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9144>
    8f20:	80 91 f3 13 	lds	r24, 0x13F3	; 0x8013f3 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c16d5>
    8f24:	90 91 19 14 	lds	r25, 0x1419	; 0x801419 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c16fb>
    8f28:	98 17       	cp	r25, r24
    8f2a:	09 f4       	brne	.+2      	; 0x8f2e <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x8f2e>
    8f2c:	6f c3       	rjmp	.+1758   	; 0x960c <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x960c>
    8f2e:	e0 91 f3 13 	lds	r30, 0x13F3	; 0x8013f3 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c16d5>
    8f32:	2e 2f       	mov	r18, r30
    8f34:	30 e0       	ldi	r19, 0x00	; 0
    8f36:	86 e0       	ldi	r24, 0x06	; 6
    8f38:	e8 9f       	mul	r30, r24
    8f3a:	f0 01       	movw	r30, r0
    8f3c:	11 24       	eor	r1, r1
    8f3e:	eb 50       	subi	r30, 0x0B	; 11
    8f40:	fc 4e       	sbci	r31, 0xEC	; 236
    8f42:	f0 93 df 0a 	sts	0x0ADF, r31	; 0x800adf <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dc1>
    8f46:	e0 93 de 0a 	sts	0x0ADE, r30	; 0x800ade <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dc0>
    8f4a:	83 81       	ldd	r24, Z+3	; 0x03
    8f4c:	94 81       	ldd	r25, Z+4	; 0x04
    8f4e:	90 93 89 00 	sts	0x0089, r25	; 0x800089 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c036b>
    8f52:	80 93 88 00 	sts	0x0088, r24	; 0x800088 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c036a>
    8f56:	80 81       	ld	r24, Z
    8f58:	91 81       	ldd	r25, Z+1	; 0x01
    8f5a:	90 93 da 0a 	sts	0x0ADA, r25	; 0x800ada <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dbc>
    8f5e:	80 93 d9 0a 	sts	0x0AD9, r24	; 0x800ad9 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dbb>
    8f62:	e2 81       	ldd	r30, Z+2	; 0x02
    8f64:	80 91 db 0a 	lds	r24, 0x0ADB	; 0x800adb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dbd>
    8f68:	8e 17       	cp	r24, r30
    8f6a:	09 f4       	brne	.+2      	; 0x8f6e <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x8f6e>
    8f6c:	4c c0       	rjmp	.+152    	; 0x9006 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9006>
    8f6e:	e0 93 db 0a 	sts	0x0ADB, r30	; 0x800adb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dbd>
    8f72:	82 e2       	ldi	r24, 0x22	; 34
    8f74:	e8 9f       	mul	r30, r24
    8f76:	f0 01       	movw	r30, r0
    8f78:	11 24       	eor	r1, r1
    8f7a:	e6 5e       	subi	r30, 0xE6	; 230
    8f7c:	fb 4e       	sbci	r31, 0xEB	; 235
    8f7e:	f0 93 dd 0a 	sts	0x0ADD, r31	; 0x800add <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dbf>
    8f82:	e0 93 dc 0a 	sts	0x0ADC, r30	; 0x800adc <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dbe>
    8f86:	85 8d       	ldd	r24, Z+29	; 0x1d
    8f88:	96 8d       	ldd	r25, Z+30	; 0x1e
    8f8a:	a7 8d       	ldd	r26, Z+31	; 0x1f
    8f8c:	b0 a1       	ldd	r27, Z+32	; 0x20
    8f8e:	b6 95       	lsr	r27
    8f90:	a7 95       	ror	r26
    8f92:	97 95       	ror	r25
    8f94:	87 95       	ror	r24
    8f96:	80 93 b5 0a 	sts	0x0AB5, r24	; 0x800ab5 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d97>
    8f9a:	90 93 b6 0a 	sts	0x0AB6, r25	; 0x800ab6 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d98>
    8f9e:	a0 93 b7 0a 	sts	0x0AB7, r26	; 0x800ab7 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d99>
    8fa2:	b0 93 b8 0a 	sts	0x0AB8, r27	; 0x800ab8 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9a>
    8fa6:	80 93 b1 0a 	sts	0x0AB1, r24	; 0x800ab1 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d93>
    8faa:	90 93 b2 0a 	sts	0x0AB2, r25	; 0x800ab2 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d94>
    8fae:	a0 93 b3 0a 	sts	0x0AB3, r26	; 0x800ab3 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d95>
    8fb2:	b0 93 b4 0a 	sts	0x0AB4, r27	; 0x800ab4 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d96>
    8fb6:	80 93 ad 0a 	sts	0x0AAD, r24	; 0x800aad <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8f>
    8fba:	90 93 ae 0a 	sts	0x0AAE, r25	; 0x800aae <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d90>
    8fbe:	a0 93 af 0a 	sts	0x0AAF, r26	; 0x800aaf <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d91>
    8fc2:	b0 93 b0 0a 	sts	0x0AB0, r27	; 0x800ab0 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d92>
    8fc6:	80 93 a9 0a 	sts	0x0AA9, r24	; 0x800aa9 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8b>
    8fca:	90 93 aa 0a 	sts	0x0AAA, r25	; 0x800aaa <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8c>
    8fce:	a0 93 ab 0a 	sts	0x0AAB, r26	; 0x800aab <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8d>
    8fd2:	b0 93 ac 0a 	sts	0x0AAC, r27	; 0x800aac <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8e>
    8fd6:	80 93 a5 0a 	sts	0x0AA5, r24	; 0x800aa5 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d87>
    8fda:	90 93 a6 0a 	sts	0x0AA6, r25	; 0x800aa6 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d88>
    8fde:	a0 93 a7 0a 	sts	0x0AA7, r26	; 0x800aa7 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d89>
    8fe2:	b0 93 a8 0a 	sts	0x0AA8, r27	; 0x800aa8 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8a>
    8fe6:	80 93 a1 0a 	sts	0x0AA1, r24	; 0x800aa1 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d83>
    8fea:	90 93 a2 0a 	sts	0x0AA2, r25	; 0x800aa2 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d84>
    8fee:	a0 93 a3 0a 	sts	0x0AA3, r26	; 0x800aa3 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d85>
    8ff2:	b0 93 a4 0a 	sts	0x0AA4, r27	; 0x800aa4 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d86>
    8ff6:	80 93 9d 0a 	sts	0x0A9D, r24	; 0x800a9d <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d7f>
    8ffa:	90 93 9e 0a 	sts	0x0A9E, r25	; 0x800a9e <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d80>
    8ffe:	a0 93 9f 0a 	sts	0x0A9F, r26	; 0x800a9f <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d81>
    9002:	b0 93 a0 0a 	sts	0x0AA0, r27	; 0x800aa0 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d82>
    9006:	e0 91 dc 0a 	lds	r30, 0x0ADC	; 0x800adc <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dbe>
    900a:	f0 91 dd 0a 	lds	r31, 0x0ADD	; 0x800add <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dbf>
    900e:	80 81       	ld	r24, Z
    9010:	90 91 9a 0a 	lds	r25, 0x0A9A	; 0x800a9a <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d7c>
    9014:	89 27       	eor	r24, r25
    9016:	80 93 bc 0a 	sts	0x0ABC, r24	; 0x800abc <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9e>
    901a:	86 e0       	ldi	r24, 0x06	; 6
    901c:	82 9f       	mul	r24, r18
    901e:	d0 01       	movw	r26, r0
    9020:	83 9f       	mul	r24, r19
    9022:	b0 0d       	add	r27, r0
    9024:	11 24       	eor	r1, r1
    9026:	ab 50       	subi	r26, 0x0B	; 11
    9028:	bc 4e       	sbci	r27, 0xEC	; 236
    902a:	15 96       	adiw	r26, 0x05	; 5
    902c:	8c 91       	ld	r24, X
    902e:	41 81       	ldd	r20, Z+1	; 0x01
    9030:	52 81       	ldd	r21, Z+2	; 0x02
    9032:	63 81       	ldd	r22, Z+3	; 0x03
    9034:	74 81       	ldd	r23, Z+4	; 0x04
    9036:	08 2e       	mov	r0, r24
    9038:	04 c0       	rjmp	.+8      	; 0x9042 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9042>
    903a:	76 95       	lsr	r23
    903c:	67 95       	ror	r22
    903e:	57 95       	ror	r21
    9040:	47 95       	ror	r20
    9042:	0a 94       	dec	r0
    9044:	d2 f7       	brpl	.-12     	; 0x903a <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x903a>
    9046:	40 93 bd 0a 	sts	0x0ABD, r20	; 0x800abd <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9f>
    904a:	50 93 be 0a 	sts	0x0ABE, r21	; 0x800abe <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da0>
    904e:	60 93 bf 0a 	sts	0x0ABF, r22	; 0x800abf <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da1>
    9052:	70 93 c0 0a 	sts	0x0AC0, r23	; 0x800ac0 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da2>
    9056:	45 81       	ldd	r20, Z+5	; 0x05
    9058:	56 81       	ldd	r21, Z+6	; 0x06
    905a:	67 81       	ldd	r22, Z+7	; 0x07
    905c:	70 85       	ldd	r23, Z+8	; 0x08
    905e:	08 2e       	mov	r0, r24
    9060:	04 c0       	rjmp	.+8      	; 0x906a <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x906a>
    9062:	76 95       	lsr	r23
    9064:	67 95       	ror	r22
    9066:	57 95       	ror	r21
    9068:	47 95       	ror	r20
    906a:	0a 94       	dec	r0
    906c:	d2 f7       	brpl	.-12     	; 0x9062 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9062>
    906e:	40 93 c1 0a 	sts	0x0AC1, r20	; 0x800ac1 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da3>
    9072:	50 93 c2 0a 	sts	0x0AC2, r21	; 0x800ac2 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da4>
    9076:	60 93 c3 0a 	sts	0x0AC3, r22	; 0x800ac3 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da5>
    907a:	70 93 c4 0a 	sts	0x0AC4, r23	; 0x800ac4 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da6>
    907e:	41 85       	ldd	r20, Z+9	; 0x09
    9080:	52 85       	ldd	r21, Z+10	; 0x0a
    9082:	63 85       	ldd	r22, Z+11	; 0x0b
    9084:	74 85       	ldd	r23, Z+12	; 0x0c
    9086:	08 2e       	mov	r0, r24
    9088:	04 c0       	rjmp	.+8      	; 0x9092 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9092>
    908a:	76 95       	lsr	r23
    908c:	67 95       	ror	r22
    908e:	57 95       	ror	r21
    9090:	47 95       	ror	r20
    9092:	0a 94       	dec	r0
    9094:	d2 f7       	brpl	.-12     	; 0x908a <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x908a>
    9096:	40 93 c5 0a 	sts	0x0AC5, r20	; 0x800ac5 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da7>
    909a:	50 93 c6 0a 	sts	0x0AC6, r21	; 0x800ac6 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da8>
    909e:	60 93 c7 0a 	sts	0x0AC7, r22	; 0x800ac7 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da9>
    90a2:	70 93 c8 0a 	sts	0x0AC8, r23	; 0x800ac8 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0daa>
    90a6:	45 85       	ldd	r20, Z+13	; 0x0d
    90a8:	56 85       	ldd	r21, Z+14	; 0x0e
    90aa:	67 85       	ldd	r22, Z+15	; 0x0f
    90ac:	70 89       	ldd	r23, Z+16	; 0x10
    90ae:	08 2e       	mov	r0, r24
    90b0:	04 c0       	rjmp	.+8      	; 0x90ba <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x90ba>
    90b2:	76 95       	lsr	r23
    90b4:	67 95       	ror	r22
    90b6:	57 95       	ror	r21
    90b8:	47 95       	ror	r20
    90ba:	0a 94       	dec	r0
    90bc:	d2 f7       	brpl	.-12     	; 0x90b2 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x90b2>
    90be:	40 93 c9 0a 	sts	0x0AC9, r20	; 0x800ac9 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dab>
    90c2:	50 93 ca 0a 	sts	0x0ACA, r21	; 0x800aca <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dac>
    90c6:	60 93 cb 0a 	sts	0x0ACB, r22	; 0x800acb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dad>
    90ca:	70 93 cc 0a 	sts	0x0ACC, r23	; 0x800acc <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dae>
    90ce:	41 89       	ldd	r20, Z+17	; 0x11
    90d0:	52 89       	ldd	r21, Z+18	; 0x12
    90d2:	63 89       	ldd	r22, Z+19	; 0x13
    90d4:	74 89       	ldd	r23, Z+20	; 0x14
    90d6:	08 2e       	mov	r0, r24
    90d8:	04 c0       	rjmp	.+8      	; 0x90e2 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x90e2>
    90da:	76 95       	lsr	r23
    90dc:	67 95       	ror	r22
    90de:	57 95       	ror	r21
    90e0:	47 95       	ror	r20
    90e2:	0a 94       	dec	r0
    90e4:	d2 f7       	brpl	.-12     	; 0x90da <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x90da>
    90e6:	40 93 cd 0a 	sts	0x0ACD, r20	; 0x800acd <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0daf>
    90ea:	50 93 ce 0a 	sts	0x0ACE, r21	; 0x800ace <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db0>
    90ee:	60 93 cf 0a 	sts	0x0ACF, r22	; 0x800acf <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db1>
    90f2:	70 93 d0 0a 	sts	0x0AD0, r23	; 0x800ad0 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db2>
    90f6:	45 89       	ldd	r20, Z+21	; 0x15
    90f8:	56 89       	ldd	r21, Z+22	; 0x16
    90fa:	67 89       	ldd	r22, Z+23	; 0x17
    90fc:	70 8d       	ldd	r23, Z+24	; 0x18
    90fe:	08 2e       	mov	r0, r24
    9100:	04 c0       	rjmp	.+8      	; 0x910a <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x910a>
    9102:	76 95       	lsr	r23
    9104:	67 95       	ror	r22
    9106:	57 95       	ror	r21
    9108:	47 95       	ror	r20
    910a:	0a 94       	dec	r0
    910c:	d2 f7       	brpl	.-12     	; 0x9102 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9102>
    910e:	40 93 d1 0a 	sts	0x0AD1, r20	; 0x800ad1 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db3>
    9112:	50 93 d2 0a 	sts	0x0AD2, r21	; 0x800ad2 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db4>
    9116:	60 93 d3 0a 	sts	0x0AD3, r22	; 0x800ad3 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db5>
    911a:	70 93 d4 0a 	sts	0x0AD4, r23	; 0x800ad4 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db6>
    911e:	41 8d       	ldd	r20, Z+25	; 0x19
    9120:	52 8d       	ldd	r21, Z+26	; 0x1a
    9122:	63 8d       	ldd	r22, Z+27	; 0x1b
    9124:	74 8d       	ldd	r23, Z+28	; 0x1c
    9126:	04 c0       	rjmp	.+8      	; 0x9130 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9130>
    9128:	76 95       	lsr	r23
    912a:	67 95       	ror	r22
    912c:	57 95       	ror	r21
    912e:	47 95       	ror	r20
    9130:	8a 95       	dec	r24
    9132:	d2 f7       	brpl	.-12     	; 0x9128 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9128>
    9134:	40 93 d5 0a 	sts	0x0AD5, r20	; 0x800ad5 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db7>
    9138:	50 93 d6 0a 	sts	0x0AD6, r21	; 0x800ad6 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db8>
    913c:	60 93 d7 0a 	sts	0x0AD7, r22	; 0x800ad7 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db9>
    9140:	70 93 d8 0a 	sts	0x0AD8, r23	; 0x800ad8 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dba>
    9144:	80 91 f2 14 	lds	r24, 0x14F2	; 0x8014f2 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c17d4>
    9148:	81 30       	cpi	r24, 0x01	; 1
    914a:	b9 f4       	brne	.+46     	; 0x917a <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x917a>
    914c:	80 91 06 01 	lds	r24, 0x0106	; 0x800106 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c03e8>
    9150:	80 78       	andi	r24, 0x80	; 128
    9152:	90 91 5c 16 	lds	r25, 0x165C	; 0x80165c <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c193e>
    9156:	89 17       	cp	r24, r25
    9158:	81 f0       	breq	.+32     	; 0x917a <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x917a>
    915a:	10 92 f2 14 	sts	0x14F2, r1	; 0x8014f2 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c17d4>
    915e:	8c e1       	ldi	r24, 0x1C	; 28
    9160:	e5 e6       	ldi	r30, 0x65	; 101
    9162:	f6 e1       	ldi	r31, 0x16	; 22
    9164:	a1 e8       	ldi	r26, 0x81	; 129
    9166:	b6 e1       	ldi	r27, 0x16	; 22
    9168:	01 90       	ld	r0, Z+
    916a:	0d 92       	st	X+, r0
    916c:	8a 95       	dec	r24
    916e:	e1 f7       	brne	.-8      	; 0x9168 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9168>
    9170:	80 91 f1 14 	lds	r24, 0x14F1	; 0x8014f1 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c17d3>
    9174:	80 64       	ori	r24, 0x40	; 64
    9176:	80 93 f1 14 	sts	0x14F1, r24	; 0x8014f1 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c17d3>
    917a:	10 92 bb 0a 	sts	0x0ABB, r1	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    917e:	40 91 9d 0a 	lds	r20, 0x0A9D	; 0x800a9d <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d7f>
    9182:	50 91 9e 0a 	lds	r21, 0x0A9E	; 0x800a9e <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d80>
    9186:	60 91 9f 0a 	lds	r22, 0x0A9F	; 0x800a9f <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d81>
    918a:	70 91 a0 0a 	lds	r23, 0x0AA0	; 0x800aa0 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d82>
    918e:	80 91 bd 0a 	lds	r24, 0x0ABD	; 0x800abd <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9f>
    9192:	90 91 be 0a 	lds	r25, 0x0ABE	; 0x800abe <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da0>
    9196:	a0 91 bf 0a 	lds	r26, 0x0ABF	; 0x800abf <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da1>
    919a:	b0 91 c0 0a 	lds	r27, 0x0AC0	; 0x800ac0 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da2>
    919e:	48 0f       	add	r20, r24
    91a0:	59 1f       	adc	r21, r25
    91a2:	6a 1f       	adc	r22, r26
    91a4:	7b 1f       	adc	r23, r27
    91a6:	40 93 9d 0a 	sts	0x0A9D, r20	; 0x800a9d <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d7f>
    91aa:	50 93 9e 0a 	sts	0x0A9E, r21	; 0x800a9e <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d80>
    91ae:	60 93 9f 0a 	sts	0x0A9F, r22	; 0x800a9f <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d81>
    91b2:	70 93 a0 0a 	sts	0x0AA0, r23	; 0x800aa0 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d82>
    91b6:	e0 91 dc 0a 	lds	r30, 0x0ADC	; 0x800adc <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dbe>
    91ba:	f0 91 dd 0a 	lds	r31, 0x0ADD	; 0x800add <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dbf>
    91be:	85 8d       	ldd	r24, Z+29	; 0x1d
    91c0:	96 8d       	ldd	r25, Z+30	; 0x1e
    91c2:	a7 8d       	ldd	r26, Z+31	; 0x1f
    91c4:	b0 a1       	ldd	r27, Z+32	; 0x20
    91c6:	84 17       	cp	r24, r20
    91c8:	95 07       	cpc	r25, r21
    91ca:	a6 07       	cpc	r26, r22
    91cc:	b7 07       	cpc	r27, r23
    91ce:	48 f5       	brcc	.+82     	; 0x9222 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9222>
    91d0:	24 e0       	ldi	r18, 0x04	; 4
    91d2:	20 93 bb 0a 	sts	0x0ABB, r18	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    91d6:	48 1b       	sub	r20, r24
    91d8:	59 0b       	sbc	r21, r25
    91da:	6a 0b       	sbc	r22, r26
    91dc:	7b 0b       	sbc	r23, r27
    91de:	40 93 9d 0a 	sts	0x0A9D, r20	; 0x800a9d <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d7f>
    91e2:	50 93 9e 0a 	sts	0x0A9E, r21	; 0x800a9e <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d80>
    91e6:	60 93 9f 0a 	sts	0x0A9F, r22	; 0x800a9f <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d81>
    91ea:	70 93 a0 0a 	sts	0x0AA0, r23	; 0x800aa0 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d82>
    91ee:	21 a1       	ldd	r18, Z+33	; 0x21
    91f0:	21 11       	cpse	r18, r1
    91f2:	17 c0       	rjmp	.+46     	; 0x9222 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9222>
    91f4:	40 91 65 16 	lds	r20, 0x1665	; 0x801665 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1947>
    91f8:	50 91 66 16 	lds	r21, 0x1666	; 0x801666 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1948>
    91fc:	60 91 67 16 	lds	r22, 0x1667	; 0x801667 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1949>
    9200:	70 91 68 16 	lds	r23, 0x1668	; 0x801668 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c194a>
    9204:	20 81       	ld	r18, Z
    9206:	21 ff       	sbrs	r18, 1
    9208:	24 c2       	rjmp	.+1096   	; 0x9652 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9652>
    920a:	41 50       	subi	r20, 0x01	; 1
    920c:	51 09       	sbc	r21, r1
    920e:	61 09       	sbc	r22, r1
    9210:	71 09       	sbc	r23, r1
    9212:	40 93 65 16 	sts	0x1665, r20	; 0x801665 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1947>
    9216:	50 93 66 16 	sts	0x1666, r21	; 0x801666 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1948>
    921a:	60 93 67 16 	sts	0x1667, r22	; 0x801667 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1949>
    921e:	70 93 68 16 	sts	0x1668, r23	; 0x801668 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c194a>
    9222:	40 91 a1 0a 	lds	r20, 0x0AA1	; 0x800aa1 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d83>
    9226:	50 91 a2 0a 	lds	r21, 0x0AA2	; 0x800aa2 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d84>
    922a:	60 91 a3 0a 	lds	r22, 0x0AA3	; 0x800aa3 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d85>
    922e:	70 91 a4 0a 	lds	r23, 0x0AA4	; 0x800aa4 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d86>
    9232:	00 91 c1 0a 	lds	r16, 0x0AC1	; 0x800ac1 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da3>
    9236:	10 91 c2 0a 	lds	r17, 0x0AC2	; 0x800ac2 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da4>
    923a:	20 91 c3 0a 	lds	r18, 0x0AC3	; 0x800ac3 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da5>
    923e:	30 91 c4 0a 	lds	r19, 0x0AC4	; 0x800ac4 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da6>
    9242:	40 0f       	add	r20, r16
    9244:	51 1f       	adc	r21, r17
    9246:	62 1f       	adc	r22, r18
    9248:	73 1f       	adc	r23, r19
    924a:	40 93 a1 0a 	sts	0x0AA1, r20	; 0x800aa1 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d83>
    924e:	50 93 a2 0a 	sts	0x0AA2, r21	; 0x800aa2 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d84>
    9252:	60 93 a3 0a 	sts	0x0AA3, r22	; 0x800aa3 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d85>
    9256:	70 93 a4 0a 	sts	0x0AA4, r23	; 0x800aa4 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d86>
    925a:	84 17       	cp	r24, r20
    925c:	95 07       	cpc	r25, r21
    925e:	a6 07       	cpc	r26, r22
    9260:	b7 07       	cpc	r27, r23
    9262:	58 f5       	brcc	.+86     	; 0x92ba <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x92ba>
    9264:	20 91 bb 0a 	lds	r18, 0x0ABB	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    9268:	20 61       	ori	r18, 0x10	; 16
    926a:	20 93 bb 0a 	sts	0x0ABB, r18	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    926e:	48 1b       	sub	r20, r24
    9270:	59 0b       	sbc	r21, r25
    9272:	6a 0b       	sbc	r22, r26
    9274:	7b 0b       	sbc	r23, r27
    9276:	40 93 a1 0a 	sts	0x0AA1, r20	; 0x800aa1 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d83>
    927a:	50 93 a2 0a 	sts	0x0AA2, r21	; 0x800aa2 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d84>
    927e:	60 93 a3 0a 	sts	0x0AA3, r22	; 0x800aa3 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d85>
    9282:	70 93 a4 0a 	sts	0x0AA4, r23	; 0x800aa4 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d86>
    9286:	21 a1       	ldd	r18, Z+33	; 0x21
    9288:	21 11       	cpse	r18, r1
    928a:	17 c0       	rjmp	.+46     	; 0x92ba <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x92ba>
    928c:	40 91 69 16 	lds	r20, 0x1669	; 0x801669 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c194b>
    9290:	50 91 6a 16 	lds	r21, 0x166A	; 0x80166a <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c194c>
    9294:	60 91 6b 16 	lds	r22, 0x166B	; 0x80166b <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c194d>
    9298:	70 91 6c 16 	lds	r23, 0x166C	; 0x80166c <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c194e>
    929c:	20 81       	ld	r18, Z
    929e:	23 ff       	sbrs	r18, 3
    92a0:	dd c1       	rjmp	.+954    	; 0x965c <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x965c>
    92a2:	41 50       	subi	r20, 0x01	; 1
    92a4:	51 09       	sbc	r21, r1
    92a6:	61 09       	sbc	r22, r1
    92a8:	71 09       	sbc	r23, r1
    92aa:	40 93 69 16 	sts	0x1669, r20	; 0x801669 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c194b>
    92ae:	50 93 6a 16 	sts	0x166A, r21	; 0x80166a <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c194c>
    92b2:	60 93 6b 16 	sts	0x166B, r22	; 0x80166b <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c194d>
    92b6:	70 93 6c 16 	sts	0x166C, r23	; 0x80166c <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c194e>
    92ba:	40 91 a5 0a 	lds	r20, 0x0AA5	; 0x800aa5 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d87>
    92be:	50 91 a6 0a 	lds	r21, 0x0AA6	; 0x800aa6 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d88>
    92c2:	60 91 a7 0a 	lds	r22, 0x0AA7	; 0x800aa7 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d89>
    92c6:	70 91 a8 0a 	lds	r23, 0x0AA8	; 0x800aa8 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8a>
    92ca:	00 91 c5 0a 	lds	r16, 0x0AC5	; 0x800ac5 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da7>
    92ce:	10 91 c6 0a 	lds	r17, 0x0AC6	; 0x800ac6 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da8>
    92d2:	20 91 c7 0a 	lds	r18, 0x0AC7	; 0x800ac7 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0da9>
    92d6:	30 91 c8 0a 	lds	r19, 0x0AC8	; 0x800ac8 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0daa>
    92da:	40 0f       	add	r20, r16
    92dc:	51 1f       	adc	r21, r17
    92de:	62 1f       	adc	r22, r18
    92e0:	73 1f       	adc	r23, r19
    92e2:	40 93 a5 0a 	sts	0x0AA5, r20	; 0x800aa5 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d87>
    92e6:	50 93 a6 0a 	sts	0x0AA6, r21	; 0x800aa6 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d88>
    92ea:	60 93 a7 0a 	sts	0x0AA7, r22	; 0x800aa7 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d89>
    92ee:	70 93 a8 0a 	sts	0x0AA8, r23	; 0x800aa8 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8a>
    92f2:	84 17       	cp	r24, r20
    92f4:	95 07       	cpc	r25, r21
    92f6:	a6 07       	cpc	r26, r22
    92f8:	b7 07       	cpc	r27, r23
    92fa:	58 f5       	brcc	.+86     	; 0x9352 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9352>
    92fc:	20 91 bb 0a 	lds	r18, 0x0ABB	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    9300:	20 64       	ori	r18, 0x40	; 64
    9302:	20 93 bb 0a 	sts	0x0ABB, r18	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    9306:	48 1b       	sub	r20, r24
    9308:	59 0b       	sbc	r21, r25
    930a:	6a 0b       	sbc	r22, r26
    930c:	7b 0b       	sbc	r23, r27
    930e:	40 93 a5 0a 	sts	0x0AA5, r20	; 0x800aa5 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d87>
    9312:	50 93 a6 0a 	sts	0x0AA6, r21	; 0x800aa6 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d88>
    9316:	60 93 a7 0a 	sts	0x0AA7, r22	; 0x800aa7 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d89>
    931a:	70 93 a8 0a 	sts	0x0AA8, r23	; 0x800aa8 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8a>
    931e:	21 a1       	ldd	r18, Z+33	; 0x21
    9320:	21 11       	cpse	r18, r1
    9322:	17 c0       	rjmp	.+46     	; 0x9352 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9352>
    9324:	40 91 6d 16 	lds	r20, 0x166D	; 0x80166d <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c194f>
    9328:	50 91 6e 16 	lds	r21, 0x166E	; 0x80166e <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1950>
    932c:	60 91 6f 16 	lds	r22, 0x166F	; 0x80166f <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1951>
    9330:	70 91 70 16 	lds	r23, 0x1670	; 0x801670 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1952>
    9334:	20 81       	ld	r18, Z
    9336:	25 ff       	sbrs	r18, 5
    9338:	96 c1       	rjmp	.+812    	; 0x9666 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9666>
    933a:	41 50       	subi	r20, 0x01	; 1
    933c:	51 09       	sbc	r21, r1
    933e:	61 09       	sbc	r22, r1
    9340:	71 09       	sbc	r23, r1
    9342:	40 93 6d 16 	sts	0x166D, r20	; 0x80166d <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c194f>
    9346:	50 93 6e 16 	sts	0x166E, r21	; 0x80166e <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1950>
    934a:	60 93 6f 16 	sts	0x166F, r22	; 0x80166f <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1951>
    934e:	70 93 70 16 	sts	0x1670, r23	; 0x801670 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1952>
    9352:	40 91 a9 0a 	lds	r20, 0x0AA9	; 0x800aa9 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8b>
    9356:	50 91 aa 0a 	lds	r21, 0x0AAA	; 0x800aaa <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8c>
    935a:	60 91 ab 0a 	lds	r22, 0x0AAB	; 0x800aab <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8d>
    935e:	70 91 ac 0a 	lds	r23, 0x0AAC	; 0x800aac <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8e>
    9362:	00 91 c9 0a 	lds	r16, 0x0AC9	; 0x800ac9 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dab>
    9366:	10 91 ca 0a 	lds	r17, 0x0ACA	; 0x800aca <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dac>
    936a:	20 91 cb 0a 	lds	r18, 0x0ACB	; 0x800acb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dad>
    936e:	30 91 cc 0a 	lds	r19, 0x0ACC	; 0x800acc <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dae>
    9372:	40 0f       	add	r20, r16
    9374:	51 1f       	adc	r21, r17
    9376:	62 1f       	adc	r22, r18
    9378:	73 1f       	adc	r23, r19
    937a:	40 93 a9 0a 	sts	0x0AA9, r20	; 0x800aa9 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8b>
    937e:	50 93 aa 0a 	sts	0x0AAA, r21	; 0x800aaa <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8c>
    9382:	60 93 ab 0a 	sts	0x0AAB, r22	; 0x800aab <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8d>
    9386:	70 93 ac 0a 	sts	0x0AAC, r23	; 0x800aac <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8e>
    938a:	84 17       	cp	r24, r20
    938c:	95 07       	cpc	r25, r21
    938e:	a6 07       	cpc	r26, r22
    9390:	b7 07       	cpc	r27, r23
    9392:	58 f5       	brcc	.+86     	; 0x93ea <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x93ea>
    9394:	20 91 bb 0a 	lds	r18, 0x0ABB	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    9398:	20 68       	ori	r18, 0x80	; 128
    939a:	20 93 bb 0a 	sts	0x0ABB, r18	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    939e:	48 1b       	sub	r20, r24
    93a0:	59 0b       	sbc	r21, r25
    93a2:	6a 0b       	sbc	r22, r26
    93a4:	7b 0b       	sbc	r23, r27
    93a6:	40 93 a9 0a 	sts	0x0AA9, r20	; 0x800aa9 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8b>
    93aa:	50 93 aa 0a 	sts	0x0AAA, r21	; 0x800aaa <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8c>
    93ae:	60 93 ab 0a 	sts	0x0AAB, r22	; 0x800aab <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8d>
    93b2:	70 93 ac 0a 	sts	0x0AAC, r23	; 0x800aac <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8e>
    93b6:	21 a1       	ldd	r18, Z+33	; 0x21
    93b8:	21 11       	cpse	r18, r1
    93ba:	17 c0       	rjmp	.+46     	; 0x93ea <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x93ea>
    93bc:	40 91 71 16 	lds	r20, 0x1671	; 0x801671 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1953>
    93c0:	50 91 72 16 	lds	r21, 0x1672	; 0x801672 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1954>
    93c4:	60 91 73 16 	lds	r22, 0x1673	; 0x801673 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1955>
    93c8:	70 91 74 16 	lds	r23, 0x1674	; 0x801674 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1956>
    93cc:	20 81       	ld	r18, Z
    93ce:	26 ff       	sbrs	r18, 6
    93d0:	4f c1       	rjmp	.+670    	; 0x9670 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9670>
    93d2:	41 50       	subi	r20, 0x01	; 1
    93d4:	51 09       	sbc	r21, r1
    93d6:	61 09       	sbc	r22, r1
    93d8:	71 09       	sbc	r23, r1
    93da:	40 93 71 16 	sts	0x1671, r20	; 0x801671 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1953>
    93de:	50 93 72 16 	sts	0x1672, r21	; 0x801672 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1954>
    93e2:	60 93 73 16 	sts	0x1673, r22	; 0x801673 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1955>
    93e6:	70 93 74 16 	sts	0x1674, r23	; 0x801674 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1956>
    93ea:	40 91 ad 0a 	lds	r20, 0x0AAD	; 0x800aad <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8f>
    93ee:	50 91 ae 0a 	lds	r21, 0x0AAE	; 0x800aae <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d90>
    93f2:	60 91 af 0a 	lds	r22, 0x0AAF	; 0x800aaf <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d91>
    93f6:	70 91 b0 0a 	lds	r23, 0x0AB0	; 0x800ab0 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d92>
    93fa:	00 91 cd 0a 	lds	r16, 0x0ACD	; 0x800acd <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0daf>
    93fe:	10 91 ce 0a 	lds	r17, 0x0ACE	; 0x800ace <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db0>
    9402:	20 91 cf 0a 	lds	r18, 0x0ACF	; 0x800acf <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db1>
    9406:	30 91 d0 0a 	lds	r19, 0x0AD0	; 0x800ad0 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db2>
    940a:	40 0f       	add	r20, r16
    940c:	51 1f       	adc	r21, r17
    940e:	62 1f       	adc	r22, r18
    9410:	73 1f       	adc	r23, r19
    9412:	40 93 ad 0a 	sts	0x0AAD, r20	; 0x800aad <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8f>
    9416:	50 93 ae 0a 	sts	0x0AAE, r21	; 0x800aae <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d90>
    941a:	60 93 af 0a 	sts	0x0AAF, r22	; 0x800aaf <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d91>
    941e:	70 93 b0 0a 	sts	0x0AB0, r23	; 0x800ab0 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d92>
    9422:	84 17       	cp	r24, r20
    9424:	95 07       	cpc	r25, r21
    9426:	a6 07       	cpc	r26, r22
    9428:	b7 07       	cpc	r27, r23
    942a:	58 f5       	brcc	.+86     	; 0x9482 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9482>
    942c:	20 91 bb 0a 	lds	r18, 0x0ABB	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    9430:	22 60       	ori	r18, 0x02	; 2
    9432:	20 93 bb 0a 	sts	0x0ABB, r18	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    9436:	48 1b       	sub	r20, r24
    9438:	59 0b       	sbc	r21, r25
    943a:	6a 0b       	sbc	r22, r26
    943c:	7b 0b       	sbc	r23, r27
    943e:	40 93 ad 0a 	sts	0x0AAD, r20	; 0x800aad <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d8f>
    9442:	50 93 ae 0a 	sts	0x0AAE, r21	; 0x800aae <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d90>
    9446:	60 93 af 0a 	sts	0x0AAF, r22	; 0x800aaf <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d91>
    944a:	70 93 b0 0a 	sts	0x0AB0, r23	; 0x800ab0 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d92>
    944e:	21 a1       	ldd	r18, Z+33	; 0x21
    9450:	21 11       	cpse	r18, r1
    9452:	17 c0       	rjmp	.+46     	; 0x9482 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9482>
    9454:	40 91 75 16 	lds	r20, 0x1675	; 0x801675 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1957>
    9458:	50 91 76 16 	lds	r21, 0x1676	; 0x801676 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1958>
    945c:	60 91 77 16 	lds	r22, 0x1677	; 0x801677 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1959>
    9460:	70 91 78 16 	lds	r23, 0x1678	; 0x801678 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c195a>
    9464:	20 81       	ld	r18, Z
    9466:	20 ff       	sbrs	r18, 0
    9468:	08 c1       	rjmp	.+528    	; 0x967a <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x967a>
    946a:	41 50       	subi	r20, 0x01	; 1
    946c:	51 09       	sbc	r21, r1
    946e:	61 09       	sbc	r22, r1
    9470:	71 09       	sbc	r23, r1
    9472:	40 93 75 16 	sts	0x1675, r20	; 0x801675 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1957>
    9476:	50 93 76 16 	sts	0x1676, r21	; 0x801676 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1958>
    947a:	60 93 77 16 	sts	0x1677, r22	; 0x801677 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1959>
    947e:	70 93 78 16 	sts	0x1678, r23	; 0x801678 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c195a>
    9482:	40 91 b1 0a 	lds	r20, 0x0AB1	; 0x800ab1 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d93>
    9486:	50 91 b2 0a 	lds	r21, 0x0AB2	; 0x800ab2 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d94>
    948a:	60 91 b3 0a 	lds	r22, 0x0AB3	; 0x800ab3 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d95>
    948e:	70 91 b4 0a 	lds	r23, 0x0AB4	; 0x800ab4 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d96>
    9492:	00 91 d1 0a 	lds	r16, 0x0AD1	; 0x800ad1 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db3>
    9496:	10 91 d2 0a 	lds	r17, 0x0AD2	; 0x800ad2 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db4>
    949a:	20 91 d3 0a 	lds	r18, 0x0AD3	; 0x800ad3 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db5>
    949e:	30 91 d4 0a 	lds	r19, 0x0AD4	; 0x800ad4 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db6>
    94a2:	40 0f       	add	r20, r16
    94a4:	51 1f       	adc	r21, r17
    94a6:	62 1f       	adc	r22, r18
    94a8:	73 1f       	adc	r23, r19
    94aa:	40 93 b1 0a 	sts	0x0AB1, r20	; 0x800ab1 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d93>
    94ae:	50 93 b2 0a 	sts	0x0AB2, r21	; 0x800ab2 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d94>
    94b2:	60 93 b3 0a 	sts	0x0AB3, r22	; 0x800ab3 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d95>
    94b6:	70 93 b4 0a 	sts	0x0AB4, r23	; 0x800ab4 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d96>
    94ba:	84 17       	cp	r24, r20
    94bc:	95 07       	cpc	r25, r21
    94be:	a6 07       	cpc	r26, r22
    94c0:	b7 07       	cpc	r27, r23
    94c2:	58 f5       	brcc	.+86     	; 0x951a <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x951a>
    94c4:	20 91 bb 0a 	lds	r18, 0x0ABB	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    94c8:	28 60       	ori	r18, 0x08	; 8
    94ca:	20 93 bb 0a 	sts	0x0ABB, r18	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    94ce:	48 1b       	sub	r20, r24
    94d0:	59 0b       	sbc	r21, r25
    94d2:	6a 0b       	sbc	r22, r26
    94d4:	7b 0b       	sbc	r23, r27
    94d6:	40 93 b1 0a 	sts	0x0AB1, r20	; 0x800ab1 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d93>
    94da:	50 93 b2 0a 	sts	0x0AB2, r21	; 0x800ab2 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d94>
    94de:	60 93 b3 0a 	sts	0x0AB3, r22	; 0x800ab3 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d95>
    94e2:	70 93 b4 0a 	sts	0x0AB4, r23	; 0x800ab4 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d96>
    94e6:	21 a1       	ldd	r18, Z+33	; 0x21
    94e8:	21 11       	cpse	r18, r1
    94ea:	17 c0       	rjmp	.+46     	; 0x951a <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x951a>
    94ec:	40 91 79 16 	lds	r20, 0x1679	; 0x801679 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c195b>
    94f0:	50 91 7a 16 	lds	r21, 0x167A	; 0x80167a <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c195c>
    94f4:	60 91 7b 16 	lds	r22, 0x167B	; 0x80167b <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c195d>
    94f8:	70 91 7c 16 	lds	r23, 0x167C	; 0x80167c <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c195e>
    94fc:	20 81       	ld	r18, Z
    94fe:	22 ff       	sbrs	r18, 2
    9500:	c1 c0       	rjmp	.+386    	; 0x9684 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9684>
    9502:	41 50       	subi	r20, 0x01	; 1
    9504:	51 09       	sbc	r21, r1
    9506:	61 09       	sbc	r22, r1
    9508:	71 09       	sbc	r23, r1
    950a:	40 93 79 16 	sts	0x1679, r20	; 0x801679 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c195b>
    950e:	50 93 7a 16 	sts	0x167A, r21	; 0x80167a <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c195c>
    9512:	60 93 7b 16 	sts	0x167B, r22	; 0x80167b <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c195d>
    9516:	70 93 7c 16 	sts	0x167C, r23	; 0x80167c <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c195e>
    951a:	40 91 b5 0a 	lds	r20, 0x0AB5	; 0x800ab5 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d97>
    951e:	50 91 b6 0a 	lds	r21, 0x0AB6	; 0x800ab6 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d98>
    9522:	60 91 b7 0a 	lds	r22, 0x0AB7	; 0x800ab7 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d99>
    9526:	70 91 b8 0a 	lds	r23, 0x0AB8	; 0x800ab8 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9a>
    952a:	00 91 d5 0a 	lds	r16, 0x0AD5	; 0x800ad5 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db7>
    952e:	10 91 d6 0a 	lds	r17, 0x0AD6	; 0x800ad6 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db8>
    9532:	20 91 d7 0a 	lds	r18, 0x0AD7	; 0x800ad7 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0db9>
    9536:	30 91 d8 0a 	lds	r19, 0x0AD8	; 0x800ad8 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dba>
    953a:	40 0f       	add	r20, r16
    953c:	51 1f       	adc	r21, r17
    953e:	62 1f       	adc	r22, r18
    9540:	73 1f       	adc	r23, r19
    9542:	40 93 b5 0a 	sts	0x0AB5, r20	; 0x800ab5 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d97>
    9546:	50 93 b6 0a 	sts	0x0AB6, r21	; 0x800ab6 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d98>
    954a:	60 93 b7 0a 	sts	0x0AB7, r22	; 0x800ab7 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d99>
    954e:	70 93 b8 0a 	sts	0x0AB8, r23	; 0x800ab8 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9a>
    9552:	84 17       	cp	r24, r20
    9554:	95 07       	cpc	r25, r21
    9556:	a6 07       	cpc	r26, r22
    9558:	b7 07       	cpc	r27, r23
    955a:	50 f5       	brcc	.+84     	; 0x95b0 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x95b0>
    955c:	20 91 bb 0a 	lds	r18, 0x0ABB	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    9560:	20 62       	ori	r18, 0x20	; 32
    9562:	20 93 bb 0a 	sts	0x0ABB, r18	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    9566:	48 1b       	sub	r20, r24
    9568:	59 0b       	sbc	r21, r25
    956a:	6a 0b       	sbc	r22, r26
    956c:	7b 0b       	sbc	r23, r27
    956e:	40 93 b5 0a 	sts	0x0AB5, r20	; 0x800ab5 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d97>
    9572:	50 93 b6 0a 	sts	0x0AB6, r21	; 0x800ab6 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d98>
    9576:	60 93 b7 0a 	sts	0x0AB7, r22	; 0x800ab7 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d99>
    957a:	70 93 b8 0a 	sts	0x0AB8, r23	; 0x800ab8 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9a>
    957e:	81 a1       	ldd	r24, Z+33	; 0x21
    9580:	81 11       	cpse	r24, r1
    9582:	16 c0       	rjmp	.+44     	; 0x95b0 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x95b0>
    9584:	80 91 7d 16 	lds	r24, 0x167D	; 0x80167d <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c195f>
    9588:	90 91 7e 16 	lds	r25, 0x167E	; 0x80167e <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1960>
    958c:	a0 91 7f 16 	lds	r26, 0x167F	; 0x80167f <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1961>
    9590:	b0 91 80 16 	lds	r27, 0x1680	; 0x801680 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1962>
    9594:	20 81       	ld	r18, Z
    9596:	24 ff       	sbrs	r18, 4
    9598:	7a c0       	rjmp	.+244    	; 0x968e <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x968e>
    959a:	01 97       	sbiw	r24, 0x01	; 1
    959c:	a1 09       	sbc	r26, r1
    959e:	b1 09       	sbc	r27, r1
    95a0:	80 93 7d 16 	sts	0x167D, r24	; 0x80167d <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c195f>
    95a4:	90 93 7e 16 	sts	0x167E, r25	; 0x80167e <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1960>
    95a8:	a0 93 7f 16 	sts	0x167F, r26	; 0x80167f <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1961>
    95ac:	b0 93 80 16 	sts	0x1680, r27	; 0x801680 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1962>
    95b0:	80 91 5f 16 	lds	r24, 0x165F	; 0x80165f <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1941>
    95b4:	84 30       	cpi	r24, 0x04	; 4
    95b6:	09 f0       	breq	.+2      	; 0x95ba <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x95ba>
    95b8:	91 c0       	rjmp	.+290    	; 0x96dc <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x96dc>
    95ba:	80 91 bb 0a 	lds	r24, 0x0ABB	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    95be:	90 91 9e 16 	lds	r25, 0x169E	; 0x80169e <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1980>
    95c2:	89 23       	and	r24, r25
    95c4:	80 93 bb 0a 	sts	0x0ABB, r24	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    95c8:	80 91 d9 0a 	lds	r24, 0x0AD9	; 0x800ad9 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dbb>
    95cc:	90 91 da 0a 	lds	r25, 0x0ADA	; 0x800ada <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dbc>
    95d0:	01 97       	sbiw	r24, 0x01	; 1
    95d2:	90 93 da 0a 	sts	0x0ADA, r25	; 0x800ada <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dbc>
    95d6:	80 93 d9 0a 	sts	0x0AD9, r24	; 0x800ad9 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dbb>
    95da:	89 2b       	or	r24, r25
    95dc:	69 f4       	brne	.+26     	; 0x95f8 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x95f8>
    95de:	10 92 df 0a 	sts	0x0ADF, r1	; 0x800adf <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dc1>
    95e2:	10 92 de 0a 	sts	0x0ADE, r1	; 0x800ade <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0dc0>
    95e6:	80 91 f3 13 	lds	r24, 0x13F3	; 0x8013f3 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c16d5>
    95ea:	8f 5f       	subi	r24, 0xFF	; 255
    95ec:	80 93 f3 13 	sts	0x13F3, r24	; 0x8013f3 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c16d5>
    95f0:	86 30       	cpi	r24, 0x06	; 6
    95f2:	11 f4       	brne	.+4      	; 0x95f8 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x95f8>
    95f4:	10 92 f3 13 	sts	0x13F3, r1	; 0x8013f3 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c16d5>
    95f8:	80 91 bb 0a 	lds	r24, 0x0ABB	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    95fc:	90 91 9b 0a 	lds	r25, 0x0A9B	; 0x800a9b <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d7d>
    9600:	89 27       	eor	r24, r25
    9602:	80 93 bb 0a 	sts	0x0ABB, r24	; 0x800abb <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d9d>
    9606:	10 92 9c 0a 	sts	0x0A9C, r1	; 0x800a9c <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c0d7e>
    960a:	0a c0       	rjmp	.+20     	; 0x9620 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9620>
    960c:	0e 94 69 2a 	call	0x54d2	; 0x54d2 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x54d2>
    9610:	9f b7       	in	r25, 0x3f	; 63
    9612:	f8 94       	cli
    9614:	80 91 f1 14 	lds	r24, 0x14F1	; 0x8014f1 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c17d3>
    9618:	84 60       	ori	r24, 0x04	; 4
    961a:	80 93 f1 14 	sts	0x14F1, r24	; 0x8014f1 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c17d3>
    961e:	9f bf       	out	0x3f, r25	; 63
