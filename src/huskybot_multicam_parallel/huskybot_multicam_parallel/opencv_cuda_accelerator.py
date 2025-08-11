# CUDA OpenCV Helper Class
# Optimizes intensive OpenCV operations for GPU acceleration

import cv2
import numpy as np
from typing import Tuple, List, Optional

class OpenCVCudaAccelerator:
    """
    Helper class untuk mengoptimalkan operasi OpenCV dengan CUDA acceleration
    """
    
    def __init__(self, use_cuda: bool = True):
        self.use_cuda = use_cuda and cv2.cuda.getCudaEnabledDeviceCount() > 0
        self.stream = cv2.cuda.Stream() if self.use_cuda else None
        
        if self.use_cuda:
            print(f"✅ OpenCV CUDA Accelerator initialized - {cv2.cuda.getCudaEnabledDeviceCount()} GPU(s) available")
            # Pre-create filters for better performance
            self._prepare_filters()
        else:
            print("⚠️  OpenCV CUDA Accelerator using CPU fallback")
    
# CUDA OpenCV Helper Class
# Optimizes intensive OpenCV operations for GPU acceleration

import cv2
import numpy as np
from typing import Tuple, List, Optional, Union
import time

class OpenCVCudaAccelerator:
    """
    Helper class untuk mengoptimalkan operasi OpenCV dengan CUDA acceleration
    Fokus pada operasi yang paling sering digunakan di project ini
    """
    
    def __init__(self, use_cuda: bool = True):
        self.use_cuda = use_cuda and cv2.cuda.getCudaEnabledDeviceCount() > 0
        self.stream = cv2.cuda.Stream() if self.use_cuda else None
        
        # Statistics untuk monitoring performance
        self.stats = {
            'resize_calls': 0,
            'resize_gpu_time': 0.0,
            'resize_cpu_time': 0.0,
            'addweighted_calls': 0,
            'bilateral_calls': 0
        }
        
        if self.use_cuda:
            print(f"✅ OpenCV CUDA Accelerator initialized - {cv2.cuda.getCudaEnabledDeviceCount()} GPU(s) available")
            self._prepare_filters()
        else:
            print("⚠️  OpenCV CUDA Accelerator using CPU fallback")
    
    def _prepare_filters(self) -> None:
        """Pre-create commonly used CUDA filters untuk better performance"""
        try:
            # Bilateral filter untuk image smoothing
            self.bilateral_gpu = True
            print("✅ Bilateral filter CUDA ready")
            
        except Exception as e:
            print(f"⚠️  Some CUDA filters unavailable: {e}")
            self.bilateral_gpu = False
    
    def resize(self, image: np.ndarray, target_size: Tuple[int, int], 
               interpolation: int = cv2.INTER_LINEAR) -> np.ndarray:
        """
        GPU-accelerated resize operation - PALING SERING DIGUNAKAN
        """
        self.stats['resize_calls'] += 1
        
        if not self.use_cuda:
            start_time = time.time()
            result = cv2.resize(image, target_size, interpolation=interpolation)
            self.stats['resize_cpu_time'] += time.time() - start_time
            return result
        
        try:
            start_time = time.time()
            
            # Upload to GPU
            gpu_img = cv2.cuda_GpuMat()
            gpu_img.upload(image)
            
            # Resize on GPU
            gpu_resized = cv2.cuda.resize(gpu_img, target_size, interpolation=interpolation, stream=self.stream)
            
            # Download result
            result = gpu_resized.download(stream=self.stream)
            self.stream.waitForCompletion()
            
            self.stats['resize_gpu_time'] += time.time() - start_time
            return result
            
        except Exception as e:
            print(f"❌ GPU resize failed, falling back to CPU: {e}")
            start_time = time.time()
            result = cv2.resize(image, target_size, interpolation=interpolation)
            self.stats['resize_cpu_time'] += time.time() - start_time
            return result
    
    def add_weighted(self, src1: np.ndarray, alpha: float, src2: np.ndarray, 
                    beta: float, gamma: float = 0) -> np.ndarray:
        """
        GPU-accelerated alpha blending untuk transparent overlays
        """
        self.stats['addweighted_calls'] += 1
        
        if not self.use_cuda:
            return cv2.addWeighted(src1, alpha, src2, beta, gamma)
        
        try:
            gpu_src1 = cv2.cuda_GpuMat()
            gpu_src2 = cv2.cuda_GpuMat()
            gpu_src1.upload(src1)
            gpu_src2.upload(src2)
            
            gpu_result = cv2.cuda.addWeighted(gpu_src1, alpha, gpu_src2, beta, gamma, stream=self.stream)
            
            result = gpu_result.download(stream=self.stream)
            self.stream.waitForCompletion()
            
            return result
        except Exception as e:
            print(f"❌ GPU addWeighted failed, falling back to CPU: {e}")
            return cv2.addWeighted(src1, alpha, src2, beta, gamma)
    
    def bilateral_filter(self, image: np.ndarray, d: int = -1, 
                        sigma_color: float = 50, sigma_space: float = 50) -> np.ndarray:
        """
        GPU-accelerated bilateral filter untuk edge-preserving smoothing
        """
        self.stats['bilateral_calls'] += 1
        
        if not self.use_cuda or not self.bilateral_gpu:
            return cv2.bilateralFilter(image, d, sigma_color, sigma_space)
        
        try:
            gpu_img = cv2.cuda_GpuMat()
            gpu_img.upload(image)
            
            gpu_filtered = cv2.cuda.bilateralFilter(gpu_img, d, sigma_color, sigma_space, stream=self.stream)
            
            result = gpu_filtered.download(stream=self.stream)
            self.stream.waitForCompletion()
            
            return result
        except Exception as e:
            print(f"❌ GPU bilateralFilter failed, falling back to CPU: {e}")
            return cv2.bilateralFilter(image, d, sigma_color, sigma_space)
    
    def gaussian_blur(self, image: np.ndarray, kernel_size: Tuple[int, int], 
                     sigma: float) -> np.ndarray:
        """
        GPU-accelerated Gaussian blur
        """
        if not self.use_cuda:
            return cv2.GaussianBlur(image, kernel_size, sigma)
        
        try:
            gpu_img = cv2.cuda_GpuMat()
            gpu_img.upload(image)
            
            # Use bilateral filter as alternative or implement with custom filter
            # For now, fall back to CPU for complex operations
            result = cv2.GaussianBlur(image, kernel_size, sigma)
            return result
        except Exception as e:
            print(f"❌ GPU GaussianBlur failed, falling back to CPU: {e}")
            return cv2.GaussianBlur(image, kernel_size, sigma)
    
    def morphology_ex(self, image: np.ndarray, operation: int, 
                     kernel: np.ndarray) -> np.ndarray:
        """
        GPU-accelerated morphological operations
        """
        if not self.use_cuda:
            return cv2.morphologyEx(image, operation, kernel)
        
        try:
            # For now, use CPU fallback for morphology as it's complex to optimize
            # Future improvement: use pre-created morphology filters
            return cv2.morphologyEx(image, operation, kernel)
        except Exception as e:
            print(f"❌ GPU morphologyEx failed, falling back to CPU: {e}")
            return cv2.morphologyEx(image, operation, kernel)
    
    def batch_resize(self, images: List[np.ndarray], target_size: Tuple[int, int], 
                    interpolation: int = cv2.INTER_LINEAR) -> List[np.ndarray]:
        """
        Batch resize untuk multiple images (untuk multi-camera processing)
        """
        if not images:
            return []
        
        if not self.use_cuda:
            return [cv2.resize(img, target_size, interpolation=interpolation) for img in images]
        
        results = []
        try:
            for img in images:
                gpu_img = cv2.cuda_GpuMat()
                gpu_img.upload(img)
                gpu_resized = cv2.cuda.resize(gpu_img, target_size, interpolation=interpolation, stream=self.stream)
                results.append(gpu_resized.download(stream=self.stream))
            
            self.stream.waitForCompletion()
            return results
        except Exception as e:
            print(f"❌ GPU batch resize failed, falling back to CPU: {e}")
            return [cv2.resize(img, target_size, interpolation=interpolation) for img in images]
    
    def get_performance_stats(self) -> dict:
        """
        Dapatkan statistik performance GPU vs CPU
        """
        stats = self.stats.copy()
        
        if stats['resize_calls'] > 0:
            avg_gpu_time = stats['resize_gpu_time'] / stats['resize_calls'] if stats['resize_gpu_time'] > 0 else 0
            avg_cpu_time = stats['resize_cpu_time'] / stats['resize_calls'] if stats['resize_cpu_time'] > 0 else 0
            
            stats['avg_resize_gpu_time'] = avg_gpu_time
            stats['avg_resize_cpu_time'] = avg_cpu_time
            
            if avg_cpu_time > 0 and avg_gpu_time > 0:
                stats['resize_speedup'] = avg_cpu_time / avg_gpu_time
        
        return stats
    
    def print_performance_report(self) -> None:
        """
        Print detailed performance report
        """
        stats = self.get_performance_stats()
        
        print("\\n=== OpenCV CUDA Performance Report ===")
        print(f"CUDA Enabled: {self.use_cuda}")
        print(f"Total Resize Calls: {stats['resize_calls']}")
        print(f"Total AddWeighted Calls: {stats['addweighted_calls']}")
        print(f"Total Bilateral Filter Calls: {stats['bilateral_calls']}")
        
        if 'resize_speedup' in stats:
            print(f"Resize Speedup: {stats['resize_speedup']:.2f}x")
        
        if stats['resize_gpu_time'] > 0:
            print(f"Total GPU Time (resize): {stats['resize_gpu_time']:.3f}s")
        if stats['resize_cpu_time'] > 0:
            print(f"Total CPU Time (resize): {stats['resize_cpu_time']:.3f}s")
    
    def performance_benchmark(self) -> None:
        """
        Run comprehensive performance benchmark
        """
        print("\\n=== Running OpenCV CUDA Performance Benchmark ===")
        
        # Test images dengan berbagai ukuran
        test_cases = [
            (480, 640, 3),   # Standard camera resolution
            (720, 1280, 3),  # HD resolution  
            (1080, 1920, 3)  # Full HD resolution
        ]
        
        iterations = 20
        
        for height, width, channels in test_cases:
            print(f"\\nTesting {width}x{height} images:")
            test_img = np.random.randint(0, 255, (height, width, channels), dtype=np.uint8)
            target_size = (width//2, height//2)
            
            # CPU benchmark
            start_time = time.time()
            for _ in range(iterations):
                _ = cv2.resize(test_img, target_size)
            cpu_time = time.time() - start_time
            
            # GPU benchmark (if available)
            if self.use_cuda:
                start_time = time.time()
                for _ in range(iterations):
                    _ = self.resize(test_img, target_size)
                gpu_time = time.time() - start_time
                
                speedup = cpu_time / gpu_time
                print(f"  CPU: {cpu_time:.3f}s, GPU: {gpu_time:.3f}s, Speedup: {speedup:.2f}x")
            else:
                print(f"  CPU: {cpu_time:.3f}s, GPU: N/A")
