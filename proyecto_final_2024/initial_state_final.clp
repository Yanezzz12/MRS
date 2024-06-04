
;************************************************
;*						*
;*	Initial state 				*
;*						*
;*                      J.Savage, UNAM          *
;*						*
;*                      1/5/2024                *
;*                                              *
;************************************************


(deffacts Initial-state-objects-rooms-robot


; Objects definitions
	( item (type Objects) (name A)(room studio))
	( item (type Objects) (name B)(room studio))
	( item (type Objects) (name C)(room studio))
	( item (type Objects) (name D)(room corridor))
	( item (type Objects) (name E)(room corridor))
	( item (type Objects) (name F)(room corridor))


; Rooms definitions
	( Room (name deposit)(center 0.70 1.51))
	( Room (name corridor)(center 0.6 1.0))
	( Room (name studio)(center 0.25 1.45))
	( Room (name service)(center 1.65 0.55))
	( Room (name kitchen)(center 1.60 1.40))
	( Room (name bedroom)(center 1.0 0.55))

; Robots definitions
	( item (type Robot) (name robot)(zone frontexit)(pose 1.048340 1.107002 0.0))

; Furniture definitions
	( item (type Furniture) (name fridge)(zone kitchen)(image fridge)( attributes no-pick white)(pose 1.50 1.436 0.0))
	( item (type Furniture) (name table)(zone service)(image table)( attributes no-pick brown)(pose 1.65 0.35 0.0))

; Doors definitions
	( item (type Door) (name outsidedoor) (status closed) )


;goal definitions
	(transfer studio F)
	(transfer bedroom E)
	(transfer corridor D)
	(transfer deposit C)
	(transfer kitchen B)
	(transfer service A)

)



