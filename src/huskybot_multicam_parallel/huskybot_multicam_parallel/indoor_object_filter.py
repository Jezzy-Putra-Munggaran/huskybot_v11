# Indoor Object Detection Filter
# Filters YOLO detections to only include objects relevant for indoor environments

class IndoorObjectFilter:
    """
    Filter untuk membatasi deteksi hanya pada objek-objek yang relevan 
    untuk indoor environment Clearpath Husky A200
    """
    
    def __init__(self):
        # COCO class indices untuk 10 objek indoor yang diizinkan
        self.allowed_classes = {
            0: 'person',
            11: 'stop sign', 
            24: 'backpack',
            26: 'handbag',
            28: 'suitcase',
            39: 'bottle',
            41: 'cup',
            56: 'chair',
            67: 'cell phone',
            63: 'laptop'
        }
        
        # Set untuk lookup cepat
        self.allowed_indices = set(self.allowed_classes.keys())
        
        # Statistics tracking
        self.total_detections = 0
        self.filtered_detections = 0
        self.allowed_detections = 0
        
        print("🏠 Indoor Object Filter initialized")
        print(f"📋 Allowed objects: {list(self.allowed_classes.values())}")
        print(f"🔢 Allowed COCO indices: {sorted(self.allowed_indices)}")
    
    def filter_detections(self, yolo_results):
        """
        Filter YOLO results untuk hanya mengembalikan objek indoor
        
        Args:
            yolo_results: Results dari YOLO model prediction
            
        Returns:
            filtered_results: Hasil yang sudah difilter
        """
        if not yolo_results or len(yolo_results) == 0:
            return []
        
        filtered_detections = []
        
        for result in yolo_results:
            if result.boxes is not None:
                boxes = result.boxes
                
                # Filter berdasarkan class
                for i in range(len(boxes)):
                    class_id = int(boxes.cls[i].item())
                    confidence = float(boxes.conf[i].item())
                    
                    self.total_detections += 1
                    
                    # Check apakah class_id termasuk dalam allowed objects
                    if class_id in self.allowed_indices:
                        # Buat detection object
                        detection = {
                            'class_id': class_id,
                            'class_name': self.allowed_classes[class_id],
                            'confidence': confidence,
                            'bbox': boxes.xyxy[i].cpu().numpy(),
                            'original_result': result
                        }
                        
                        # Add mask if available (untuk segmentation)
                        if hasattr(result, 'masks') and result.masks is not None:
                            detection['mask'] = result.masks.data[i].cpu().numpy()
                        else:
                            detection['mask'] = None
                            
                        filtered_detections.append(detection)
                        self.allowed_detections += 1
                    else:
                        self.filtered_detections += 1
        
        return filtered_detections
    
    def is_allowed_class(self, class_id):
        """
        Check apakah class_id diizinkan
        """
        return class_id in self.allowed_indices
    
    def get_class_name(self, class_id):
        """
        Get nama class dari class_id jika diizinkan
        """
        return self.allowed_classes.get(class_id, 'unknown')
    
    def get_statistics(self):
        """
        Get statistik filtering
        """
        filter_rate = (self.filtered_detections / max(1, self.total_detections)) * 100
        
        stats = {
            'total_detections': self.total_detections,
            'allowed_detections': self.allowed_detections,
            'filtered_detections': self.filtered_detections,
            'filter_rate_percent': filter_rate,
            'allowed_classes': len(self.allowed_classes)
        }
        
        return stats
    
    def print_statistics(self):
        """
        Print statistik filtering
        """
        stats = self.get_statistics()
        
        print(f"\\n📊 Indoor Object Filter Statistics:")
        print(f"   Total detections processed: {stats['total_detections']}")
        print(f"   Allowed (indoor objects): {stats['allowed_detections']}")
        print(f"   Filtered out (non-indoor): {stats['filtered_detections']}")
        print(f"   Filter rate: {stats['filter_rate_percent']:.1f}%")
        print(f"   Allowed classes: {stats['allowed_classes']}")
    
    def reset_statistics(self):
        """
        Reset statistik tracking
        """
        self.total_detections = 0
        self.filtered_detections = 0
        self.allowed_detections = 0
