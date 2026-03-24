// ==========================================
// Presentation Controller
// ==========================================

(function () {
  'use strict';

  const slides = document.querySelectorAll('.slide');
  const totalSlides = slides.length;
  let currentSlide = 0;

  const btnPrev = document.getElementById('btn-prev');
  const btnNext = document.getElementById('btn-next');
  const slideCounter = document.getElementById('slide-counter');
  const dotsContainer = document.getElementById('slide-dots');
  const btnFullscreen = document.getElementById('btn-fullscreen');

  // ---- Create dots ----
  function createDots() {
    for (let i = 0; i < totalSlides; i++) {
      const dot = document.createElement('button');
      dot.classList.add('slide-dot');
      if (i === 0) dot.classList.add('active');
      dot.title = `Slide ${i + 1}`;
      dot.addEventListener('click', () => goToSlide(i));
      dotsContainer.appendChild(dot);
    }
  }

  // ---- Go to slide ----
  function goToSlide(index) {
    if (index < 0 || index >= totalSlides || index === currentSlide) return;

    const direction = index > currentSlide ? 1 : -1;

    // Remove active from current
    slides[currentSlide].classList.remove('active');
    slides[currentSlide].classList.add(direction > 0 ? 'prev' : '');

    // Clean prev class after transition
    setTimeout(() => {
      slides[currentSlide === index ? 0 : currentSlide].classList.remove('prev');
    }, 600);

    currentSlide = index;

    // Reset all slides
    slides.forEach((s, i) => {
      s.classList.remove('active', 'prev');
      if (i === currentSlide) {
        s.classList.add('active');
      }
    });

    updateUI();
  }

  function nextSlide() {
    if (currentSlide < totalSlides - 1) goToSlide(currentSlide + 1);
  }

  function prevSlide() {
    if (currentSlide > 0) goToSlide(currentSlide - 1);
  }

  // ---- Update UI ----
  function updateUI() {
    slideCounter.textContent = `${currentSlide + 1} / ${totalSlides}`;
    btnPrev.disabled = currentSlide === 0;
    btnNext.disabled = currentSlide === totalSlides - 1;

    // Update dots
    const dots = dotsContainer.querySelectorAll('.slide-dot');
    dots.forEach((dot, i) => {
      dot.classList.toggle('active', i === currentSlide);
    });
  }

  // ---- Keyboard navigation ----
  document.addEventListener('keydown', (e) => {
    switch (e.key) {
      case 'ArrowRight':
      case 'ArrowDown':
      case ' ':
      case 'PageDown':
        e.preventDefault();
        nextSlide();
        break;
      case 'ArrowLeft':
      case 'ArrowUp':
      case 'PageUp':
        e.preventDefault();
        prevSlide();
        break;
      case 'Home':
        e.preventDefault();
        goToSlide(0);
        break;
      case 'End':
        e.preventDefault();
        goToSlide(totalSlides - 1);
        break;
      case 'f':
      case 'F':
        if (!e.ctrlKey && !e.metaKey) {
          e.preventDefault();
          toggleFullscreen();
        }
        break;
      case 'Escape':
        if (document.fullscreenElement) {
          document.exitFullscreen();
        }
        break;
    }
  });

  // ---- Touch support ----
  let touchStartX = 0;
  let touchStartY = 0;

  document.addEventListener('touchstart', (e) => {
    touchStartX = e.changedTouches[0].screenX;
    touchStartY = e.changedTouches[0].screenY;
  }, { passive: true });

  document.addEventListener('touchend', (e) => {
    const dx = e.changedTouches[0].screenX - touchStartX;
    const dy = e.changedTouches[0].screenY - touchStartY;

    if (Math.abs(dx) > Math.abs(dy) && Math.abs(dx) > 50) {
      if (dx < 0) nextSlide();
      else prevSlide();
    }
  }, { passive: true });

  // ---- Button clicks ----
  btnPrev.addEventListener('click', prevSlide);
  btnNext.addEventListener('click', nextSlide);

  // ---- Fullscreen ----
  function toggleFullscreen() {
    if (!document.fullscreenElement) {
      document.documentElement.requestFullscreen().catch(() => {});
    } else {
      document.exitFullscreen();
    }
  }

  btnFullscreen.addEventListener('click', toggleFullscreen);

  document.addEventListener('fullscreenchange', () => {
    btnFullscreen.textContent = document.fullscreenElement ? '⛶ Exit' : '⛶ Fullscreen';
  });

  // ---- Mouse wheel (optional) ----
  let wheelTimeout = false;
  document.addEventListener('wheel', (e) => {
    if (wheelTimeout) return;
    wheelTimeout = true;
    setTimeout(() => { wheelTimeout = false; }, 600);

    if (e.deltaY > 0) nextSlide();
    else if (e.deltaY < 0) prevSlide();
  }, { passive: true });

  // ---- Init ----
  createDots();
  updateUI();

})();
