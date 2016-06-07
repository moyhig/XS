(define k-gyro-angle  (/ 750 100)) ; 7.5
(define k-gyro-speed  (/ 115 100)) ; 1.15

(define k-steer (/ 25 100)) ;  0.25

(define k-pos   (/  7 100)) ;  0.07
(define k-drive (/ -2 100)) ; -0.02
(define k-speed (/ 10 100)) ;  0.1

(define k-ratio-wheel (/ 9 10)) ; 1.0

(define t-interval 0)
(define init-interval   ())
(define update-interval ())

(define package-interval
  (let* ((t-start 0)
	 (c-loop  0))    
    (set! init-interval
	  (lambda ()
	    (set! c-loop 0)
	    (set! t-start (time))
	    (set! t-interval (/ 55 10000)))) ; 0.0055 (5.5ms)
    (set! update-interval
	  (lambda () 
	    (set! c-loop (+ c-loop 1))
	    (set! t-interval (/ (- (time) t-start)
				(* c-loop 1000)))))))

(define gyro-speed 0)
(define gyro-angle 0)
(define set-gyro-offset  ())
(define update-gyro-data ())

(define package-gyro
  (let* ((offset-samples 100)
	 (ema-offset (/ 5 10000)) ; 5
	 (gyro-offset 0))
    (set! set-gyro-offset
	  (lambda ()			;; GetGyroOffset => SetGyroOffset
	    (set! gyro-angle 0)
	    (set! gyro-speed 0)
	    (let ((g-sum 0))
	      (let loop ()
		   (set! g-sum (/ 0 1))
		   (let ((g-min  1000)
			 (g-max -1000))
		     (let g-smpl ((n offset-samples))
			  (if (<= n 0)
			      '()
			    (let ((g (sensor-raw 1)))
			      (if (> g g-max)
				  (set! g-max g))
			      (if (< g g-min)
				  (set! g-min g))
			      (set! g-sum (+ g-sum g))
			      (msleep 5)
			      (g-smpl (- n 1)))))
		     (if (> (- g-max g-min) 3)
			 (loop))))
	      (set! gyro-offset (- (/ g-sum offset-samples) 1)))))
    (set! update-gyro-data
	  (lambda ()			;; GetGyroData => UpdateGyroData
	    (let ((gyro-raw (sensor-raw 1)))
	      (set! gyro-offset (+ (* ema-offset gyro-raw)
				   (* (- 1 ema-offset) gyro-offset)))
	      (set! gyro-speed (* 1 (- gyro-raw gyro-offset)))
	      (set! gyro-angle (+ gyro-angle
				  (* gyro-speed t-interval))))))))

(define motor-pos   0)
(define motor-speed 0)
(define motor-diff  0)
(define init-motor-data   ())
(define update-motor-data ())

(define package-motor
  (let* ((mrc-sum      0)
	 (mrc-sum-prev 0)
	 (mrc-delta-p3 0)
	 (mrc-delta-p2 0)
	 (mrc-delta-p1 0))
    (set! init-motor-data
	  (lambda ()
	    (set! motor-pos   0)
	    (set! motor-speed 0)
	    (set! motor-diff  0)
	    (set! mrc-sum 0)
	    (set! mrc-sum-prev 0)
	    (set! mrc-delta-p3 0)
	    (set! mrc-delta-p2 0)
	    (set! mrc-delta-p1 0)
	    (rotation :c 0)
	    (rotation :a 0)))
    (set! update-motor-data
	  (lambda ()					;; GetMotorData
	    (let ((mrc-left  (rotation :c))
		  (mrc-right (rotation :a)))
	      (set! mrc-sum-prev mrc-sum)
	      (set! mrc-sum (+ mrc-left mrc-right))
	      (set! motor-diff (- mrc-left mrc-right))
	      (let ((mrc-delta (- mrc-sum mrc-sum-prev)))
		(set! motor-pos   (+ motor-pos mrc-delta))
		(set! motor-speed (/ (+ mrc-delta    mrc-delta-p1
					mrc-delta-p2 mrc-delta-p3)
				     (* 4 t-interval)))
		(set! mrc-delta-p3 mrc-delta-p2)
		(set! mrc-delta-p2 mrc-delta-p1)
		(set! mrc-delta-p1 mrc-delta)))))))

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
; 	               (set! motor-control-drive (*  100 control-speed)))
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
(define (ht-balance)
  (cls)
;  (rs485-gets)
;  (rs485-gets)
;  (rs485-gets)
  (puts "wait ")
  (set-gyro-offset)
;  (write (ftoi (set-gyro-offset)))
;  (write-string ":wake in 3s:")
  (puts "wake ")
  (sleep 3000)
  (init-interval)
  (init-motor-data)
  (init-motor-control)
  (catch 'end
    (let* ((t-motor-pos-ok (time))
	   (balance
	    (lambda ()
	      (update-interval)
	      (update-gyro-data)
	      (update-motor-data)
	      (set! motor-pos (- motor-pos
				 (* motor-control-drive t-interval)
				 (/ 50 100))) ;; adjust drift: larger for fwd
	      (let ((power (+ (/ (+ (* k-gyro-speed gyro-speed)
				    (* k-gyro-angle gyro-angle))
				 k-ratio-wheel)
			      (* k-pos   motor-pos)
			      (* k-drive motor-control-drive)
			      (* k-speed motor-speed))))
		(if (and (< (ftoi power) 100) (> (ftoi power) -100))
		    (set! t-motor-pos-ok (time)))
		(let ((steer (get-steer-control power)))
		  (speed :c (car steer))
		  (speed :a (cdr steer)))))))
      (with-watcher (((every :b  200)
		      (or
		      ;(update-motor-control-by-color)
		      ;(update-motor-control-by-eopd)
		      ;(update-motor-control-by-sonar)
		      ;(update-motor-control-by-touch)
		      ;(update-motor-control-by-kbd)
		       (update-motor-control-by-accel)
		       (update-motor-control)))
		     ((every :c 1000)
		      (puts "alive")
		      (if (> (- (time) t-motor-pos-ok) 1500)
			  (throw 'end 0))))
        (let loop ()
	     (balance)
	     (every :a 8 #t) ;; blocking cyclic timer
	     (loop)))))
  (speed :a (speed :c 0))
  (every :b  100 #t) ;; flush semaphore
  (every :c 1000 #t) ;; flush semaphore
  (puts "fail ")
  (ftoi (* t-interval 1000000)))

;(ht-balance)
