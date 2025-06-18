frame_clahe_gray = cv.cvtColor(frame_clahe, cv.COLOR_BGR2GRAY)
            if not hasattr(self, 'prev_frame') or self.prev_frame is None:
                self.prev_frame = frame_clahe_gray
                self.prev_points = cv.goodFeaturesToTrack(self.prev_frame, mask=None, **self.feature_params)
                flow_frame = frame.copy()
            else:
                # Calculate optical flow
                next_points, status, error = cv.calcOpticalFlowPyrLK(self.prev_frame, frame_clahe_gray, self.prev_points, None, **self.lk_params)
                if next_points is not None:
                    good_new = next_points[status == 1]
                    good_old = self.prev_points[status == 1]

                    # Draw the tracks
                    mask = np.zeros_like(frame)
                    for i, (new, old) in enumerate(zip(good_new, good_old)):
                        a, b = new.ravel()
                        c, d = old.ravel()
                        mask = cv.line(mask, (int(a), int(b)), (int(c), int(d)), (0, 255, 0), 2)
                        frame = cv.circle(frame, (int(a), int(b)), 5, (0, 0, 255), -1)

                    flow_frame = cv.add(frame, mask)

                    # Update previous frame and points
                    self.prev_frame = frame_clahe_gray.copy()
                    self.prev_points = good_new.reshape(-1, 1, 2)
                else:
                    flow_frame = frame.copy()
            
            cv.imshow('Optical Flow', flow_frame)
            cv.waitKey(1)