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

