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

