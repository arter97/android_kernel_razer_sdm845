old_way: (only flash boot.img and erase dtbo)
	boot.img :
		sdm845-v2-mtp_aop-evb.dts
			sdm845-v2-mtp_aop-evb.dtsi
				sdm845-fih-memmap.dtsi
				sdm845-fih-apr.dtsi
				sdm845-fih-info_aop-evb.dtsi
				... add your dtsi at this
	dtbo.img : this is fake, no add dtsi at this
		sdm845-v2-mtp-fake_aop-evb.dts
			sdm845-v2-mtp-fake_aop-evb.dtsi

overlay: (flash boot.img and dtbo.img)
	boot.img : common modificatin at this
		sdm845-v2_fih.dts
			sdm845-v2_fih.dtsi
				sdm845-fih-memmap.dtsi
				sdm845-fih-apr.dtsi
	dtbo.img : custom modificatin at this
		sdm845-v2-mtp-overlay_aop-evb.dts
			sdm845-v2-mtp-overlay_aop-evb.dtsi
				sdm845-fih-info_aop-evb.dtsi
				... add your dtsi at this
