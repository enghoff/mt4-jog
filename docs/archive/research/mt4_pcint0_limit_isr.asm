
d:\mt4\backups\mt4_flash.elf:     file format elf32-avr


Disassembly of section .data:

00009956 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9956>:
    9956:	1f 92       	push	r1
    9958:	0f 92       	push	r0
    995a:	0f b6       	in	r0, 0x3f	; 63
    995c:	0f 92       	push	r0
    995e:	11 24       	eor	r1, r1
    9960:	0b b6       	in	r0, 0x3b	; 59
    9962:	0f 92       	push	r0
    9964:	2f 93       	push	r18
    9966:	3f 93       	push	r19
    9968:	4f 93       	push	r20
    996a:	5f 93       	push	r21
    996c:	6f 93       	push	r22
    996e:	7f 93       	push	r23
    9970:	8f 93       	push	r24
    9972:	9f 93       	push	r25
    9974:	af 93       	push	r26
    9976:	bf 93       	push	r27
    9978:	ef 93       	push	r30
    997a:	ff 93       	push	r31
    997c:	80 91 5f 16 	lds	r24, 0x165F	; 0x80165f <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1941>
    9980:	81 30       	cpi	r24, 0x01	; 1
    9982:	71 f0       	breq	.+28     	; 0x99a0 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x99a0>
    9984:	80 91 5d 16 	lds	r24, 0x165D	; 0x80165d <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c193f>
    9988:	81 11       	cpse	r24, r1
    998a:	0a c0       	rjmp	.+20     	; 0x99a0 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x99a0>
    998c:	0e 94 d0 2a 	call	0x55a0	; 0x55a0 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x55a0>
    9990:	9f b7       	in	r25, 0x3f	; 63
    9992:	f8 94       	cli
    9994:	80 91 5d 16 	lds	r24, 0x165D	; 0x80165d <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c193f>
    9998:	83 60       	ori	r24, 0x03	; 3
    999a:	80 93 5d 16 	sts	0x165D, r24	; 0x80165d <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c193f>
    999e:	9f bf       	out	0x3f, r25	; 63
    99a0:	ff 91       	pop	r31
    99a2:	ef 91       	pop	r30
    99a4:	bf 91       	pop	r27
    99a6:	af 91       	pop	r26
    99a8:	9f 91       	pop	r25
    99aa:	8f 91       	pop	r24
    99ac:	7f 91       	pop	r23
    99ae:	6f 91       	pop	r22
    99b0:	5f 91       	pop	r21
    99b2:	4f 91       	pop	r20
    99b4:	3f 91       	pop	r19
    99b6:	2f 91       	pop	r18
    99b8:	0f 90       	pop	r0
    99ba:	0b be       	out	0x3b, r0	; 59
    99bc:	0f 90       	pop	r0
    99be:	0f be       	out	0x3f, r0	; 63
    99c0:	0f 90       	pop	r0
    99c2:	1f 90       	pop	r1
    99c4:	18 95       	reti
    99c6:	cf 93       	push	r28
    99c8:	20 91 2e 17 	lds	r18, 0x172E	; 0x80172e <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a10>
    99cc:	30 91 2f 17 	lds	r19, 0x172F	; 0x80172f <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a11>
    99d0:	40 91 30 17 	lds	r20, 0x1730	; 0x801730 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a12>
    99d4:	50 91 31 17 	lds	r21, 0x1731	; 0x801731 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a13>
    99d8:	60 91 62 17 	lds	r22, 0x1762	; 0x801762 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a44>
    99dc:	70 91 63 17 	lds	r23, 0x1763	; 0x801763 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a45>
    99e0:	80 91 64 17 	lds	r24, 0x1764	; 0x801764 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a46>
    99e4:	90 91 65 17 	lds	r25, 0x1765	; 0x801765 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a47>
    99e8:	0e 94 41 5c 	call	0xb882	; 0xb882 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0xb882>
    99ec:	81 11       	cpse	r24, r1
    99ee:	28 c0       	rjmp	.+80     	; 0x9a40 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9a40>
    99f0:	20 91 32 17 	lds	r18, 0x1732	; 0x801732 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a14>
    99f4:	30 91 33 17 	lds	r19, 0x1733	; 0x801733 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a15>
    99f8:	40 91 34 17 	lds	r20, 0x1734	; 0x801734 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a16>
    99fc:	50 91 35 17 	lds	r21, 0x1735	; 0x801735 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a17>
    9a00:	60 91 66 17 	lds	r22, 0x1766	; 0x801766 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a48>
    9a04:	70 91 67 17 	lds	r23, 0x1767	; 0x801767 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a49>
    9a08:	80 91 68 17 	lds	r24, 0x1768	; 0x801768 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a4a>
    9a0c:	90 91 69 17 	lds	r25, 0x1769	; 0x801769 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a4b>
    9a10:	0e 94 41 5c 	call	0xb882	; 0xb882 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0xb882>
    9a14:	81 11       	cpse	r24, r1
    9a16:	14 c0       	rjmp	.+40     	; 0x9a40 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0x9a40>
    9a18:	c1 e0       	ldi	r28, 0x01	; 1
    9a1a:	20 91 36 17 	lds	r18, 0x1736	; 0x801736 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a18>
    9a1e:	30 91 37 17 	lds	r19, 0x1737	; 0x801737 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a19>
    9a22:	40 91 38 17 	lds	r20, 0x1738	; 0x801738 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a1a>
    9a26:	50 91 39 17 	lds	r21, 0x1739	; 0x801739 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a1b>
    9a2a:	60 91 6a 17 	lds	r22, 0x176A	; 0x80176a <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a4c>
    9a2e:	70 91 6b 17 	lds	r23, 0x176B	; 0x80176b <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a4d>
    9a32:	80 91 6c 17 	lds	r24, 0x176C	; 0x80176c <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a4e>
    9a36:	90 91 6d 17 	lds	r25, 0x176D	; 0x80176d <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_end+0x7c1a4f>
    9a3a:	0e 94 41 5c 	call	0xb882	; 0xb882 <_binary_d__mt4_backups_mt4_flash_2026_07_02_bin_start+0xb882>
    9a3e:	81 11       	cpse	r24, r1
    9a40:	c0 e0       	ldi	r28, 0x00	; 0
    9a42:	8c 2f       	mov	r24, r28
    9a44:	cf 91       	pop	r28
    9a46:	08 95       	ret
    9a48:	4f 92       	push	r4
    9a4a:	5f 92       	push	r5
    9a4c:	6f 92       	push	r6
    9a4e:	7f 92       	push	r7
