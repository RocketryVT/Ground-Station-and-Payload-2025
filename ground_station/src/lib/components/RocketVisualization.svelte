<script>
    export let angles = { left: 0, right: 0 };
  </script>
  
  <style>
    .flap-angles {
      display: flex;
      justify-content: space-around;
      align-items: center;
      height: 100%;
    }
  
    .flap {
      width: 45%;
      height: 100%;
      background: #2196f3;
      transition: transform 0.3s;
    }
  </style>
  
  <div class="flap-angles">
    <div class="flap" style="transform: rotate({angles.left}deg)"></div>
    <div class="flap" style="transform: rotate({angles.right}deg)"></div>
  </div>