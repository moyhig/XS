(define motor-control-drive 0)
(define motor-control-steer 0)
(define init-motor-control   ())
(define update-motor-control ())
(define get-steer-control    ())

(define package-ir-receive
  (let* ((control-speed (/ 400 100))
	 (motor-diff-target 0))
    (set-sensor-lowspeed 2)
    (set! init-motor-control
	  (lambda ()
	    (set! motor-control-drive 0)
	    (set! motor-control-steer 0)
	    (set! motor-diff-target   0)))
    (set! update-motor-control
	  (lambda ()			;; taskControl() => SetMotorControl
	    (let* ((ir (sensor-raw 2 2 #t))
		   (ir-left  (pop ir))
		   (ir-right (pop ir)))
	      (if (<= ir-left  -128) (set! ir-left  0))
	      (if (<= ir-right -128) (set! ir-right 0))
	    ; (set! ir-left  (+ ir-left  50))
	    ; (set! ir-right (+ ir-right 25))
	      (set! motor-control-drive (* (+ ir-left ir-right)
					   control-speed))
	      (set! motor-control-steer (* (- ir-left ir-right)
					   control-speed)))))
    (set! get-steer-control
	  (lambda (power)		;; SteerControl
	    (set! motor-diff-target (+ motor-diff-target
				       (* motor-control-steer t-interval)))
	    (let* ((power-steer (* k-steer
				   (- motor-diff-target motor-diff)))
		   (power-left  (ftoi (+ power power-steer)))
		   (power-right (ftoi (- power power-steer))))
	      (if (> power-left   100) (set! power-left   100))
	      (if (< power-left  -100) (set! power-left  -100))
	      (if (> power-right  100) (set! power-right  100))
	      (if (< power-right -100) (set! power-right -100))
	      (cons power-left power-right))))))

; (define update-motor-control-by-kbd ())
; 
; (define package-kbd
;   (let ((control-speed (/ 50 200)))
;     (set! update-motor-control-by-kbd
; 	  (lambda ()
; 	    (let ((kbd (read-char)))
; 	      (if (> kbd 0)
; 		  (begin
; 		   (if (= kbd 104)
; 		       (set! motor-control-steer (* -100 control-speed)))
; 		   (if (= kbd 106)
; 		       (set! motor-control-drive (* -100 control-speed)))
; 		   (if (= kbd 107)
; 		       (set! motor-control-drive (*  100 control-speed)))
; 		   (if (= kbd 108)
; 		       (set! motor-control-steer (*  100 control-speed))))
; 		#f))))))
; 
; (define update-motor-control-by-touch ())
; 
; (define package-kbd
;   (let ((control-speed (/ 100 200)))
;     (set! update-motor-control-by-touch
; 	  (lambda ()
; 	    (let ((touch (- 1023 (sensor-raw 4))))
; 	      (if (> touch 0)
; 		  (set! motor-control-steer (* touch control-speed))
; 		#f))))))
; 
; ; (define update-motor-control-by-sound ())
; ; 
; ; (define package-sound
; ;   (let ((control-speed (/ 400 200)))
; ;     (set! update-motor-control-by-sound
; ; 	  (lambda ()
; ; 	    (let ((sound-left  (- 1023 (sensor-raw 1)))
; ; 		  (sound-right (- 1023 (sensor-raw 4))))
; ; 	      (if (or (> (ftoi (sqrt sound-left )) 14) 
; ;                     (> (ftoi (sqrt sound-right)) 14))
; ; 		  (begin
; ; 		   (set! motor-control-drive (* (sqrt (+ sound-left sound-right))
; ; 						2 control-speed))
; ; 		   (set! motor-control-steer (* (- sound-left sound-right)
; ; 						control-speed)))
; ; 		#f))))))

(define update-motor-control-by-accel ())

(define package-accel
  (let ((control-speed 400))
    (set-sensor-lowspeed 3)
    (set! update-motor-control-by-accel
	  (lambda ()			;; taskControl() => SetMotorControl
	    (let ((accel (rs485-gets))) ; (sensor-raw 3 6)))
	      (if (null? accel)
		  #f
		(let* ((accel-x (pop accel))
		       (accel-y (pop accel))
		       (accel-z (pop accel)))
		  (if (> (logshr accel-x 7) 0)
		      (set! accel-x (* -1 (logxor accel-x 255))))
		  (if (> (logshr accel-y 7) 0)
		      (set! accel-y (* -1 (logxor accel-y 255))))
		  (if (> (logshr accel-z 7) 0)
		      (set! accel-z (* -1 (logxor accel-z 255))))
		  (set! accel-x (logior (logshl accel-x 2) (pop accel)))
		  (set! accel-y (logior (logshl accel-y 2) (pop accel)))
		  (set! accel-z (logior (logshl accel-z 2) (pop accel)))
		  (set! motor-control-drive (* accel-z -1
					       (/ control-speed 75)))
		  (set! motor-control-steer (* accel-y -1
					       (/ control-speed 75))))))))))

; ; (define update-motor-control-by-sonar ())
; ; 
; ; (define package-sonar
; ;   (let ((control-speed (/ 700 200))
; ;         (p-gain (/ 10 1))
; ;         (d-gain (/ 10 1000))
; ;         (sonar-prev 0)
; ;         (delta (list 0 0 0)))
; ;     (set-sensor-lowspeed 2)
; ;     (set! update-motor-control-by-sonar
; ;           (lambda ()
; ;             (let ((sonar0 (car (sensor-raw 2 1)))
; ;                   (sonar1 sonar-prev))
; ; 	      (set! sonar-prev sonar0)
; ; 	      (let* ((d0 (- sonar0 sonar1))
; ; 		     (d1 (pop delta))
; ; 		     (d2 (pop delta))
; ; 		     (d3 (pop delta)))
; ; 		(set! delta (list d0 d1 d2))
; ; 		(if (< sonar0 20)
; ; 		    (set! motor-control-drive
; ; 			  (* (+ (* p-gain (- 20 sonar0))
; ; 				(* d-gain (/ (+ d0 d1 d2 d3)
; ; 					     (* 4 t-interval))))
; ; 			     -1 control-speed))
; ; 		  #f)))))))
;  
; (define update-motor-control-by-eopd ())
; 
; (define package-eopd
;   (let ((control-speed (/ 300 200))
; 	(p-gain (/ 10 1))
; 	(d-gain (/ 10 1000))
; 	(eopd-prev 0)
; 	(delta (list 0 0 0)))
;     (light-on 1)
;     (set! update-motor-control-by-eopd
; 	  (lambda ()
; 	    (let ((eopd0 (sqrt (* (- 1023 (sensor-raw 1)) 10)))
; 		  (eopd1 eopd-prev))
; 	      (set! eopd-prev eopd0)
; 	      (let* ((d0 (- eopd0 eopd1))
; 		     (d1 (pop delta))
; 		     (d2 (pop delta))
; 		     (d3 (pop delta)))
; 		(set! delta (list d0 d1 d2))
; 		(if (> (ftoi eopd0) 15)
; 		    (set! motor-control-steer
; 			  (* (+ (* p-gain eopd0)
; 				(* d-gain (/ (+ d0 d1 d2 d3)
; 					     (* 4 t-interval))))
; 			     -1 control-speed))
; 		  #f)))))))
; 
