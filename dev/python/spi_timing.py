t_sradmotbr = 35
t_srr = 20

spi_speed = 8			# MHz
t_sck = 1/spi_speed				# (us)

t_br = 4*8*t_sck + t_sradmotbr + t_srr

print(f"burst time = {t_br} us")
